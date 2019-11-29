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
    virtual int channels() = 0;
    virtual double value(int chan) = 0;
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

struct MonoSteppable : public Steppable
{
    double val;
    virtual int channels() override { return 1; }
    virtual double value(int chan) override { return val; };
};
    
struct StereoSteppable : public Steppable
{
    double valL, valR;
    virtual int channels() override { return 2; }
    virtual double value(int chan) override { if( chan == 0 ) return valL; else return valR; };
};
    
struct Modulator : MonoSteppable
{
};

struct ConstantModulator : Modulator {
    ConstantModulator( double v ) { val = v; }
    virtual void step() override { }
    virtual bool isActive() override { return true; }
};

struct LFOSin : Modulator {
    double freq, phase, dphase;
    LFOSin( double freq ) {
        freq = freq;
        phase = 0;
        dphase = freq / srate;
    }
    virtual void step() override {
        phase += dphase;
        val = sin( phase );
    }
    virtual bool isActive() override { return true; }
};

struct ModulatorBinder : Modulator {
    std::shared_ptr<MonoSteppable> mod;
    std::function<void(double)> toThis;
    ModulatorBinder( std::shared_ptr<Modulator> mod,
                     std::function<void(double)> toThis ) {
        this->mod = mod;
        this->toThis = toThis;
        children.insert(this->mod);
    }

    virtual void step() override {
        toThis(mod->value(0));
    }
    virtual bool isActive() override {
        return mod->isActive();
    }
};
    

/*
** dPhase generators (pitch generators)
*/
struct Pitch : public MonoSteppable
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
struct Osc : public StereoSteppable
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
        valL = evaluateAtPhase(0);
        valR = evaluateAtPhase(1);
    }

    virtual double evaluateAtPhase(int chan) = 0;
};

struct SinOsc : public Osc {
    virtual double evaluateAtPhase(int chan) override {
        return sin( 2.0 * M_PI * phase );
    }
};

struct PWMOsc : public Osc {
    double pw;
    virtual void setPulseWidth( double pw ) { this->pw = pw; }
    virtual double evaluateAtPhase(int chan) override {
        auto res = 1.0;
        if( phase > pw )
            res = -1.0;
        return res;
    }
};

/*
** Envelopes
*/
struct Env : public Modulator
{
};

struct ConstantTimedEnv : public Env {
    double time, amp;
    double currtime;
    ConstantTimedEnv( double time, double amp ) : time(time), amp(amp), currtime(0) {
    }
    virtual bool isActive() override { return currtime < time; };
    virtual void step() override { currtime += stime; }
    virtual double value(int chan) override { return amp; } 
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
    virtual double value(int chan) override { return amp * val; } 
};

struct Filter : StereoSteppable
{
    std::shared_ptr<StereoSteppable> input;
    void setInput(std::shared_ptr<StereoSteppable> i) {
        if( input )
            children.erase(input);
        input = i;
        children.insert(input);
    }
};

#if 0
strict BiquadFilter : Filter // Thanks to http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
{
    float inp[3], out[3];
    float b[3], a[3];
    BiquadFilter() {
        resetBuffers();
    }

    void resetBuffers() {
        for( auto i=0; i<3; ++i )
        {
            inp[i] = 0.;
            out[i] = 0.;
        }
    }
    
    virtual void step() {
        for( auto i=0; i<2; ++i )
        {
            inp[i+1] = inp[i];
            out[i+1] = out[i];
        }
        inp[0] = input->value();

        double res;
        for( int i=0; i<3; ++i )
        {
            res += b[i] * inp[i];
            if( i != 0 )
                res += a[i] * out[i];
        }
        out[0] = res;
        val = res;
    }

    void LPF( double freq, double Q )
    {
        auto alpha = sin(freq) / ( 2.0 * Q );
        auto cosw  = cos(freq);
        auto a0 = 1 + alpha;
        
        b[0] = ( 1.0 - cosw ) / 2.0 / a0;
        b[1] = ( 1.0 - cos2 ) / a0;
        b[2] = b[0];
        a[1] = ( - 2.0 * cos2 ) / a0;
        a[2] = ( 1.0 - alpha ) / a0;
    }
}
#endif


struct Note : public StereoSteppable
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
        for( auto c=0; c<2; ++c )
        {
            auto ns = source->value(c);
            auto en = env->value(c);

            if( c == 0 )
                valL = en * ns;
            else
                valR = en * ns;
        }
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

        for( auto cs = 0; cs < nsamples; cs += 2 )
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

            double resL = 0, resR = 0;
            for( auto n : activeNotes )
            {
                resL += n->value(0);
                resR += n->value(1);
            }
            s[cs] = resL;
            s[cs+1] = resR;

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

    auto makeNote = [&p](double len, double amp, double freq) {
                        std::shared_ptr<Env> e( new ADSRHeldForTimeEnv( 0.07, 0.05, .9, 0.2, len, amp ));
                        std::shared_ptr<Pitch> ptc(new ConstantPitch(freq));
                        std::shared_ptr<PWMOsc> o(new PWMOsc());
                        o->setPitch(ptc);

                        std::shared_ptr<LFOSin> lfo(new LFOSin( 8.0 ));
                        std::shared_ptr<ModulatorBinder> b(new ModulatorBinder(lfo,
                                                                               [o](double v) {
                                                                                   auto npw = 0.5 + 0.3 * v;
                                                                                   o->setPulseWidth(npw);
                                                                               } ) );
                        o->children.insert(b);
                        
                        std::shared_ptr<Note> n(new Note( o, e ));
                        return n;
                    };

    #if 0
    auto scale = { 60, 62, 64, 65, 67, 69, 71, 72, 74 };
    auto s = 0.0;
    for(auto n : scale)
    {
        p.addNoteAtTime(s, makeNote( 0.5, 0.3, noteToFreq(n) ) );
        s += 0.2;
    }
    #endif

    p.addNoteAtTime( 0.0, makeNote( 1.8, 0.3, noteToFreq(48) ) );
    // p.addNoteAtTime( 0.2, makeNote( 1.8, 0.3, noteToFreq(48 + 5) ) );

    size_t nsamp = (size_t)( 2 * 2.5 * srate );
    double music[ nsamp ];
    p.generateSamples(music, nsamp );


    SF_INFO sfinfo;
    sfinfo.channels = 2;
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
