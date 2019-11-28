#include <iostream>
#include <iomanip>
#include <map>
#include <unordered_set>

/*
** The only thing I use which isn't std c++14 is libsndfile just because
** writing a wav would have added extra time
*/
#include "sndfile.h"

double srate = 44100;
double stime = 1.0 / srate;

struct Steppable
{
    virtual void step() = 0;
    double val;
    virtual double value() { return val; };
    virtual bool isActive() = 0;

    bool dirty = false;
    std::unordered_set<std::shared_ptr<Steppable>> children;

    virtual void advanceGraph() {
        makeDirty();
        stepIfDirty();
    }
    void makeDirty() {
        dirty = true;
        for( auto c : children )
            c->makeDirty();
    }
    void stepIfDirty() {
        for( auto c : children )
            c->stepIfDirty();

        if( dirty )
            step();
        
        dirty = false;
    }
};

struct Modulator : Steppable
{
};

struct ConstantModulator : Modulator {
    ConstantModulator( double v ) { val = v; }
    virtual void step() override { }
};

/*
** dPhase generators (pitch generators)
*/
struct Pitch : public Steppable
{
    virtual double dphase() = 0;
};

struct ConstantPitch : public Pitch
{
    double dp;
    ConstantPitch( double freqInHz ) {
        dp = freqInHz / srate;
    }
    virtual double dphase() override { return dp; }
    virtual void step() override {}
    virtual bool isActive() override { return true; }
};

/*
** Oscillators consume dPhase and make a waveform
*/
struct Osc : public Steppable
{
    virtual bool isActive() override { return true; }
    virtual void setPitch(std::shared_ptr<Pitch> p)
    {
        if( pitch )
            children.erase(pitch);
        
        pitch = p;
        children.insert(p);
    }

    std::shared_ptr<Pitch> pitch;
    float phase = 0;
    virtual void step() override {
        float dp = 0.01;
        if( pitch )
            dp = pitch->dphase();
        phase += dp;
        if( phase > 1 ) phase -= 1;
        val = evaluateAtPhase();
    }

    virtual double evaluateAtPhase() = 0;
};

struct SinOsc : public Osc {
    virtual double evaluateAtPhase() override {
        return sin( 2.0 * M_PI * phase );
    }
};

struct PWMOsc : public Osc {
    std::shared_ptr<Modulator> pwmModulator;
    void setPWMModulator( std::shared_ptr<Modulator> m ) {
        if( pwmModulator )
            children.erase( pwmModulator );
        
        pwmModulator = m;
        children.insert( pwmModulator );
    }
    virtual double evaluateAtPhase() override {
        auto pw = 0.5;
        if( pwmModulator )
            pw = pwmModulator->value();
        // A gross square for now
        auto res = 1.0;
        if( phase > pw )
            res = -1.0;
        return res;
    }
};

/*
** Envelopes
*/
struct Env : public Steppable
{
};

struct ConstantTimedEnv : public Env {
    double time, amp;
    double currtime;
    ConstantTimedEnv( double time, double amp ) : time(time), amp(amp), currtime(0) {
    }
    virtual bool isActive() override { return currtime < time; };
    virtual void step() override { currtime += stime; }
    virtual double value() override { return amp; } 
};

struct ADSRHeldForTimeEnv : public Env {
    double a, d, s, r;
    double time, amp;
    double currtime;
    ADSRHeldForTimeEnv( double a, double d, double s, double r, double time, double amp ) : time(time), amp(amp), currtime(0),
                                                                                          a(a), d(d), s(s), r(r) {
    }
    virtual bool isActive() override { return currtime < time + r; };
    virtual void step() override {
        if( currtime < a )
        {
            auto intoA = currtime / a;
            val = 1.0 * intoA;
        }
        else if( currtime < a + d )
        {
            auto intoD = (currtime - a) / d;
            val = ( 1.0 - s ) * ( 1.0 - intoD ) + s;
        }
        else if( currtime < time )
        {
            val = s;
        }
        else
        {
            auto intoR = ( currtime - time ) / r; // ( 0 for release, 1 for end)
            val = s * ( 1 - intoR );
        }
        currtime += stime;
    }
    virtual double value() override { return amp * val; } 
};

struct Note : public Steppable
{
    std::shared_ptr<Osc> source;
    std::shared_ptr<Env> env;
    Note( std::shared_ptr<Osc> isource,
          std::shared_ptr<Env> ienvelope ) {
        source = isource;
        env = ienvelope;

        children.insert(source);
        children.insert(env);
    }

    virtual void step() override {
        auto ns = source->value();
        auto en = env->value();
        
        val = en * ns;
    }

    virtual bool isActive() override {
        return env->isActive();
    }
};

struct Player {
    std::map<size_t, std::shared_ptr<Note>> sequence;
    
    void addNoteAtSample(size_t sample, std::shared_ptr<Note> n) {
        sequence[sample] = n;
    }
    void addNoteAtTime(float time, std::shared_ptr<Note> n) {
        auto sample = (size_t)( time * srate );
        sequence[sample] = n;
    }

    void generateSamples( double *s, size_t nsamples ) {
        std::unordered_set<std::shared_ptr<Note>> activeNotes;

        for( auto cs = 0; cs < nsamples; ++cs )
        {
            auto maybenrenote = sequence.find(cs);
            if( maybenrenote != sequence.end() )
            {
                activeNotes.insert( maybenrenote->second );
            }

            for( auto n : activeNotes )
                n->makeDirty();
            for( auto n : activeNotes )
                n->stepIfDirty();

            double res = 0;
            for( auto n : activeNotes )
                res += n->value();
            s[cs] = res;

            std::unordered_set<std::shared_ptr<Note>> del;
            for( auto n : activeNotes )
                if( ! n->isActive() )
                    del.insert( n );
            for( auto n : del )
                activeNotes.erase(n);
                
        }
    }
};

float noteToFreq(int n)
{
    auto n0 = n - 69; // so 440 == 0
    auto f = 440.0 * pow( 2.0, n0 / 12.0 );
    return f;
}

int main( int arcgc, char **argv )
{
    std::cout << "Disquiet 0413" << std::endl;
    Player p;

    auto makeNote = [](double len, double amp, double freq) {
                        std::shared_ptr<Env> e( new ADSRHeldForTimeEnv( 0.07, 0.05, .9, 0.2, len, amp ));
                        std::shared_ptr<Pitch> ptc(new ConstantPitch(freq));
                        std::shared_ptr<Osc> o(new PWMOsc());
                        o->setPitch(ptc);
                        std::shared_ptr<Note> n(new Note( o, e ));
                        return n;
                    };

    auto scale = { 60, 62, 64, 65, 67, 69, 71, 72 };
    auto s = 0.0;
    for(auto n : scale)
    {
        p.addNoteAtTime(s, makeNote( 0.5, 0.3, noteToFreq(n) ) );
        s += 0.2;
    }

    size_t nsamp = (size_t)( 2.5 * srate );
    double music[ nsamp ];
    p.generateSamples(music, nsamp );


    SF_INFO sfinfo;
    sfinfo.channels = 1;
    sfinfo.samplerate = srate;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
    SNDFILE* of = sf_open("example.wav", SFM_WRITE, &sfinfo);
    float sfm[ nsamp ];
    for( auto i=0; i<nsamp; ++i )
        sfm[i] = music[i];
    sf_write_float( of, &sfm[0], nsamp );
    sf_write_sync(of);
    sf_close(of);
}
