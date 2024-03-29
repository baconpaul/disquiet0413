#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <unordered_set>
#include <atomic>
#include <typeinfo>

/*
** The only thing I use which isn't std c++14 is libsndfile just because
** writing a wav would have added extra time
*/
#include "sndfile.h"

double srate = 44100 * 2;
double stime = 1.0 / srate;

static std::atomic<int> steppableCount;

struct Steppable
{
    Steppable() {
        steppableCount++;
        //std::cout << "Steppable ctor:  [" << steppableCount << "] [" << this << "]" << std::endl;
    }
    ~Steppable() {
        steppableCount--;
        //std::cout << "Steppable dtor:  [" << steppableCount << "] [" << this << "]" << std::endl;
        for( auto c : children )
        {
            //std::cout << "  - " << c.get() << " " << c.use_count() << " " << c->className() << std::endl;
        }
        children.clear();
    }
    virtual void step() = 0;
    virtual int channels() = 0;
    virtual double value(int chan) = 0;
    virtual bool isActive() = 0;
    virtual std::string className() { return "Steppable [override]"; }
    
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
        {
            step();
        }
        
        dirty = false;
    }

    void debugInfo(std::string pfx = "|-") {
        std::cout << pfx << " this=" << this << " " << className() << std::endl;
        for( auto c : children )
            c->debugInfo( pfx + "-|-" );
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
    virtual std::string className() override { return "LFOSin"; }
};

struct ModulatorBinder : Modulator {
    std::shared_ptr<MonoSteppable> mod;
    std::function<void(double)> toThis;
    ModulatorBinder( std::shared_ptr<Modulator> mod,
                     std::function<void(double)> toThis ) {
        this->mod = mod;
        this->toThis = toThis;
        children.insert(mod);
    }

    virtual void step() override {
        toThis(mod->value(0));
    }
    virtual bool isActive() override {
        return mod->isActive();
    }
    virtual std::string className() override { return "ModluatorBinder"; }
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
    virtual std::string className() override { return "ConstantPitch"; }
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
    float phase = 0, dphase = 0.01;
    virtual void step() override {
        if( pitch )
            dphase = pitch->dphase();
        phase += dphase;
        if( phase > 1 ) {
            phase -= 1;
            rollPhase();
        }
               
        valL = evaluateAtPhase(0);
        valR = evaluateAtPhase(1);
    }

    virtual void rollPhase() { }
    virtual double evaluateAtPhase(int chan) = 0;
};

struct SinOsc : public Osc {
    virtual double evaluateAtPhase(int chan) override {
        return sin( 2.0 * M_PI * phase );
    }
};

struct PWMOsc : public Osc {
    double pw = 0.5;
    double falloff = 0.2;
    bool secondHalf = false;

    virtual double evaluateAtPhase(int chan) override {
        double vphase;
        double s;
        if( phase < pw && ! secondHalf )
        {
            vphase = phase / pw;
            s = 1;
        }
        else
        {
            secondHalf = true;
            vphase = (phase - pw) / ( 1.0 - pw );
            s = -1;
        }
        auto res = s * ( falloff * ( vphase * vphase ) + 1.0 - falloff );
        
        return res;
    }
    virtual void rollPhase() override { secondHalf = false; }
    virtual std::string className() override { return "PWMOsc"; }
};

struct ThreeOscFM : public Osc {
    // 2 + 3 -> 1
    double fmul1 = 3.0, famp1 = 3.0, fmul2 = 6.0, famp2 = 1.0;
    double phase1, phase2;

    virtual void step() override {
        if( pitch )
            dphase = pitch->dphase();

        phase1 += dphase * fmul1;
        if( phase1 > 1 ) phase1 -= 1;

        phase2 += dphase * fmul2;
        if( phase2 > 1 ) phase2 -= 1;

        auto impfreq = dphase * srate * ( 1  + famp1 * sin(2.0 * M_PI * phase1) + famp2 * sin(2.0 * M_PI * phase2) );
        auto impdphase = impfreq / srate;
        phase += impdphase;
        if( phase > 1 ) phase -= 1;
        
        valL = sin(2.0 * M_PI * phase);
        valR = valL;
    }
    virtual double evaluateAtPhase(int c) override { return 0; } // skip this since I force it in step above

    virtual bool isActive() override { return true; }
    virtual std::string className() override { return "ThreeOscFM"; }
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
    double currtime = 0;
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
        else if( currtime < time + r )
        {
            auto intoR = ( currtime - time ) / r; // ( 0 for release, 1 for end)
            val = s * ( 1 - intoR );
        }
        else
        {
            val = 0;
        }
        currtime += stime;
    }
    virtual double value(int chan) override { return amp * val; }
    virtual std::string className() override { return "ADSREnv"; }
};

struct Filter : StereoSteppable
{
    std::shared_ptr<StereoSteppable> input;
    void setInput(std::shared_ptr<StereoSteppable> i) {
        if( input )
            children.erase(input);
        input = i;
        children.insert(i);
    }
    virtual bool isActive() { return input->isActive(); }
};

struct BiquadFilter : Filter // Thanks to http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
{
    double freq = 200, Q = 0.2;
    double inp[2][3], out[2][3];
    double b[3], a[3];
    BiquadFilter() {
        resetBuffers();
    }

    void resetBuffers() {
        for( auto i=0; i<3; ++i )
        {
            inp[0][i] = 0.;
            out[0][i] = 0.;
            inp[1][i] = 0.;
            out[1][i] = 0.;
        }
    }
    
    virtual void step() {
        for( auto chan=0; chan < 2; ++chan )
        {
            for( auto i=0; i<2; ++i )
            {
                inp[chan][i+1] = inp[chan][i];
                out[chan][i+1] = out[chan][i];
            }
            inp[chan][0] = input->value(chan);
            
            double res = 0.0;
            for( int i=0; i<3; ++i )
            {
                res += b[i] * inp[chan][i];
                if( i != 0 )
                    res -= a[i] * out[chan][i];
            }
            out[chan][0] = res;
        };
        valL = out[0][0];
        valR = out[1][1];
    }

    void setFreq( double f )
    {
        freq = f;
        recalcCoeffs();
    }

    void setResonance( double q )
    {
        Q = q;
        recalcCoeffs();
    }

    virtual void recalcCoeffs() = 0;
    
};

struct LPFBiquad : BiquadFilter {
    virtual void recalcCoeffs() override
    {
        auto omega = 2.0 * M_PI * freq / srate;
        auto alpha = sin(omega) / ( 2.0 * Q );
        auto cosw  = cos(omega);
        auto a0 = 1 + alpha;
        
        b[0] = ( 1.0 - cosw ) / 2.0 / a0;
        b[1] = ( 1.0 - cosw ) / a0;
        b[2] = b[0];
        a[1] = ( - 2.0 * cosw ) / a0;
        a[2] = ( 1.0 - alpha ) / a0;

    }
    virtual std::string className() override { return "LPFBiquad"; }
};


struct VCA : public StereoSteppable
{
    std::shared_ptr<StereoSteppable> source;
    std::shared_ptr<Modulator> env;
    VCA( std::shared_ptr<StereoSteppable> isource,
          std::shared_ptr<Modulator> ienvelope ) {
        source = isource;
        env = ienvelope;

        children.insert(isource);
        children.insert(ienvelope);
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

    virtual std::string className() override { return "VCA"; }
};

struct UniformMixer : StereoSteppable {
    virtual void step() override {
        double tr = 0, tl = 0;
        for( auto c : children )
        {
            tl += c->value(0);
            tr += c->value(1);
        }
        valL = tl;
        valR = tr;
    }
    virtual std::string className() override { return "UniformMixer"; }
    virtual bool isActive() override { return true; } // this could be better obvs
};

struct WeightedMixer : StereoSteppable {
    std::vector< std::pair< double, std::shared_ptr<StereoSteppable> > > subs;
    void addInput( double w,  std::shared_ptr<StereoSteppable> s ) {
        subs.push_back(std::make_pair(w, s));
        children.insert(s);
    }
    
    virtual void step() override {
        double tr = 0, tl = 0;
        for( auto c : subs )
        {
            tl += c.first * c.second->value(0);
            tr += c.first * c.second->value(1);
        }
        valL = tl;
        valR = tr;
    }
    virtual std::string className() override { return "WeightedMixer"; }
    virtual bool isActive() override { return true; } // this could be better obvs
};

struct PanningMixer : StereoSteppable {
    double pan;
    PanningMixer( double pan ) { // -1 move all left; 0 no change; 1 move all right
        this->pan = pan;
    }

    virtual void step() override {
        double tr = 0, tl = 0;
        for( auto c : children )
        {
            auto l = c->value(0);
            auto r = c->value(1);

            if( pan < 0 )
            {
                tl += l + r * (-pan);
                tr += r * ( 1 + pan);
            }
            else
            {
                tl += l * ( 1 - pan );
                tr += r + l * pan;
            }
        }
        valL = tl;
        valR = tr;
    }
    virtual std::string className() override { return "UniformMixer"; }
    virtual bool isActive() override { return true; } // this could be better obvs

};

struct DigitalDelay : StereoSteppable {
    double fb = 0.2;
    size_t nsamp;
    double *buffer;
    size_t pos;

    DigitalDelay(size_t s) {
        nsamp = s;
        buffer = new double[2*nsamp];
        for( int i=0; i<2*nsamp; ++i )
            buffer[i] = 0.0;
        pos = 0;
    }
    ~DigitalDelay() {
        delete[] buffer;
    }
    virtual void step() override {
        auto il = pos * 2;
        auto ir = pos * 2 + 1;
        auto fbl = buffer[il];
        auto fbr = buffer[ir];

        if( children.size() > 1 )
            std::cout << "Please only add one child to a delay thanks. Use a mixer first" << std::endl;

        auto k = *(children.begin());
        valL = buffer[il];
        valR = buffer[ir];
        
        buffer[il] = ( k->value(0) + fb * fbl );
        buffer[ir] = ( k->value(1) + fb * fbr );

        pos++;
        if( pos >= nsamp ) pos = 0;
    }
    virtual bool isActive() override { return true; };
    virtual std::string className() override { return "Digital Delay"; }
    
};

struct SequencePlayer : StereoSteppable {
    std::map<size_t, std::shared_ptr<StereoSteppable>> sequence;
    size_t rendered_until = 0;

    std::shared_ptr<StereoSteppable> notePlayer;

    SequencePlayer() {
        notePlayer = std::make_shared<UniformMixer>();
        children.insert(notePlayer);
    }
    
    void addNoteAtSample(size_t sample, std::shared_ptr<StereoSteppable> n) {
        sequence[sample] = n;
    }
    void addNoteAtTime(double time, std::shared_ptr<StereoSteppable> n) {
        auto sample = (size_t)( time * srate );
        sequence[sample] = n;
    }

    virtual void step() override {
        auto maybenrenote = sequence.find(rendered_until);
        if( maybenrenote != sequence.end() )
        {
            notePlayer->children.insert( maybenrenote->second );
        }
        rendered_until ++;
        
        valL = notePlayer->value(0);
        valR = notePlayer->value(1);

        std::unordered_set<std::shared_ptr<Steppable>> del;
        for( auto n : notePlayer->children )
            if( ! n->isActive() )
                del.insert( n );
        for( auto n : del )
            notePlayer->children.erase(n);
    }

    virtual bool isActive() override { return true; }
    virtual std::string className() override { return "Sequence Player"; }
};

struct Renderer {
    std::shared_ptr<StereoSteppable> source;
    Renderer(std::shared_ptr<StereoSteppable> isource) {
        source = isource;
    }

    void generateSamples( double *s, size_t nsamples ) {
        for( auto cs = 0; cs < nsamples; cs += 2 )
        {
            source->makeDirty();
            source->stepIfDirty();
            s[cs] = source->value(0);
            s[cs+1] = source->value(1);
        }
    }

    void writeToFile( std::string fname, int nSeconds ) {
        size_t nsamp = srate * 2;
        double music[ nsamp ];


        SF_INFO sfinfo;
        sfinfo.channels = 2;
        sfinfo.samplerate = srate;
        sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
        SNDFILE* of = sf_open(fname.c_str(), SFM_WRITE, &sfinfo);

        std::cout << "Rendering source to file '" << fname << "'" << std::endl;
        
        for( int i=0; i<nSeconds; ++i )
        {
            std::cout << "  Rendering second " << std::setw(3) << i << "/" << nSeconds << std::endl;
            generateSamples( music, nsamp );
            float sfm[ nsamp ];
            for( auto i=0; i<nsamp; ++i )
            {
                sfm[i] = music[i];
                if( sfm[i] > 1.0 || sfm[i] < -1.0 )
                    std::cout << "CLIP at sample " << i << " " << sfm[i] << std::endl;
            }
            sf_write_float( of, &sfm[0], nsamp );
            sf_write_sync(of);
        }
        std::cout << "Rendering complete" << std::endl;
        sf_close(of);

    }
};

double noteToFreq(double n)
{
    auto n0 = n - 69; // so 440 == 0
    auto f = 440.0 * pow( 2.0, n0 / 12.0 );
    return f;
}

std::vector<std::tuple<int, double, double>> voicesTheme() {
    std::vector<double> n = { 60, 3, 0.7,
                              67, 1, 0.75,
                              67, 4, 0.7,
                              
                              65, 3, 0.75,
                              74, 1, 0.8,
                              74, 4, 0.85,
                              
                              72, 3, 0.85,
                              68, 1, 0.4,
                              70, 1, 0.45,
                              72, 1, 0.5,
                              74, 1, 0.6,
                              76, 1, 0.7,
                              
                              78, 3, 0.72,
                              79, 1, 0.75,
                              81, 4, 0.85,
                              
                              83, 3, 0.85,
                              72, 1, 0.6,
                              72, 4, 0.55,
                              
                              73, 3, 0.75,
                              62, 1, 0.5,
                              62, 4, 0.45,
                              
                              63, 3, 0.75,
                              52, 1, 0.45,
                              54, 1, 0.5,
                              56, 1, 0.55,
                              58, 1, 0.6,
                              60, 1, 0.67,
                              
                              62, 3, 0.72,
                              63, 1, 0.7,
                              64, 4, 0.68,
                              
                              66, 4.5, 0.7,
                              60, .5, 0.45,
                              61, .5, 0.47,
                              62, .5, 0.5,
                              
                              63, .5, 0.53,
                              64, .5, 0.55,
                              65, .5, 0.58,
                              66, .5, 0.6,
                              
                              67, 2.5, 0.75,
                              58, .5, 0.5,
                              59, .5, 0.52,
                              60, .5, 0.55,

                              61, .5, 0.57,
                              62, .5, 0.6,
                              63, .5, 0.63,
                              64, .5, 0.65,
                              
                              65, .5, 0.67,
                              66, .5, 0.7,
                              67, .5, 0.73,
                              68, .5, 0.75,

                              69, 3, 0.78,
                              64, 1, 0.7,
                              71, 3, 0.83,
                              64, 1, 0.7,
                              74, 8, 0.87,

                              72, 3, 0.75,
                              67, 1, 0.64,
                              67, 4, 0.6,

                              68, 3, 0.7,
                              63, 1, 0.6,
                              63, 4, 0.54,

                              64, 3, 0.68,
                              55, 1, 0.6,
                              57, 2, 0.62,
                              53, 2, 0.55,

                              43, 8, 0.5
    };

    std::vector<std::tuple<int, double, double>> res;
    for( auto i=0; i<n.size(); i += 3 )
    {
        res.push_back( std::make_tuple( n[i], 1.0 * n[i+1], n[i+2] ) );
    }
    return res;             
}

int runDisquiet0413()
{
    std::cout << "Disquiet 0413" << std::endl;
    auto p = std::make_shared<SequencePlayer>();

    auto makeThemeNote = [&p](double len, double amp, double freq) {
                             auto e = std::make_shared<ADSRHeldForTimeEnv>(0.07, 0.05, .9, 0.2, len, amp );
                             auto ptcL = std::make_shared<ConstantPitch>(freq * 0.99);
                             auto oL = std::make_shared<PWMOsc>();
                             oL->falloff = 0.8;
                             oL->setPitch(ptcL);
                             auto panL = std::make_shared<PanningMixer>(-0.6);
                             panL->children.insert(oL);
                             
                             auto ptcR = std::make_shared<ConstantPitch>(freq * 1.01);
                             auto oR = std::make_shared<PWMOsc>();
                             oR->setPitch(ptcR);
                             auto panR = std::make_shared<PanningMixer>(0.6);
                             panR->children.insert(oR);

                             auto o = std::make_shared<UniformMixer>();
                             o->children.insert(panR);
                             o->children.insert(panL);
                             

                             auto lfo = std::make_shared<LFOSin>(4.0);
                             std::weak_ptr<PWMOsc> wo = oL; // don't cause a cycle!
                             auto b = std::make_shared<ModulatorBinder>(lfo,
                                                                        [wo](double v) {
                                                                            auto npw = 0.5 + 0.05 * v;
                                                                            wo.lock()->pw = npw;
                                                                        } );
                        
                             oL->children.insert(b);

                             std::weak_ptr<PWMOsc> wr = oR; // don't cause a cycle!
                             auto bR = std::make_shared<ModulatorBinder>(lfo,
                                                                        [wr](double v) {
                                                                            auto npw = 0.5 - 0.05 * v;
                                                                            wr.lock()->pw = npw;
                                                                        } );
                        
                             oR->children.insert(bR);

                             auto fenv = std::make_shared<ADSRHeldForTimeEnv>( 0.05, 0.1, 0.1, 0.2, len, 1.0 );
                             auto lpf = std::make_shared<LPFBiquad>();
                             lpf->setFreq(1000);
                             lpf->setResonance(0.2);
                             lpf->setInput(o);
                             std::weak_ptr<LPFBiquad> wlpf = lpf;
                             auto lb = std::make_shared<ModulatorBinder>(fenv,
                                                                         [wlpf, freq](double v)
                                                                             {
                                                                                 wlpf.lock()->setFreq(freq * 3 + v * 1000 );
                                                                             }
                                 );
                             lpf->children.insert(lb);

                             auto mix2 = std::make_shared<UniformMixer>();
                             auto pithC = std::make_shared<ConstantPitch>(freq);
                             auto oS = std::make_shared<ThreeOscFM>();
                             oS->setPitch(pithC);
                             auto fmenv = std::make_shared<ADSRHeldForTimeEnv>( 0.01, 0.1, 0.3, 0.4, len / 2, 1.0 );
                             std::weak_ptr<ThreeOscFM> w3ow = oS;
                             auto fmb = std::make_shared<ModulatorBinder>(fmenv,
                                                                          [w3ow](double v)
                                                                              {
                                                                                  auto w3o = w3ow.lock();
                                                                                  w3o->famp1 = 3.8 * v;
                                                                                  w3o->famp2 = 1.2 * v;
                                                                                  w3o->fmul1 = 3.0 + 0.1 * v;
                                                                                  w3o->fmul2 = 5.95 + 0.1 * v;
                                                                              }
                                 );
                             oS->children.insert(fmb);

                             mix2->children.insert(lpf);
                             mix2->children.insert(oS);
                             
                             auto n = std::make_shared<VCA>( mix2, e );

                             return n;
                         };

    auto theme = voicesTheme();

    size_t csample = 0;
    double bpm = 126;
    double secondsPerBeat = 60 / bpm;
    double samplesPerBeat = secondsPerBeat * srate;
    std::cout << "spb = " << samplesPerBeat << std::endl;
    
    for( auto n : theme )
    {
        auto midinote = std::get<0>(n);
        auto dur = std::get<1>(n) / 2.0;
        auto vel = std::get<2>(n);
        auto tdur = dur * secondsPerBeat;
        auto sdur = dur * samplesPerBeat;

        p->addNoteAtSample(csample, makeThemeNote( tdur * 0.8, 0.6 * vel, noteToFreq(midinote) ));
        csample += sdur;
    }

    // OK so now build some fx. lets delay a slightly LPFed version of the main theme
    auto del = std::make_shared<DigitalDelay>( samplesPerBeat / 2 );
    auto mainlpf = std::make_shared<LPFBiquad>();
    mainlpf->setFreq(3600);
    mainlpf->setResonance(0.8);
    mainlpf->setInput(p);
    del->children.insert(mainlpf);
    del->fb = 0.4;

    // Lets hold a pulsing drone beneath the whole thing
    auto dmix = std::make_shared<WeightedMixer>();
    for( int i=0; i<3; ++i )
    {
        auto fdiff = ( i - 1.0 ) * 0.1;
        auto pan   = ( 1 - 1.0 ) * 0.7;
        
        auto drone = std::make_shared<PWMOsc>();
        auto dronePitch = std::make_shared<ConstantPitch>(noteToFreq( 60 - 12 - 12 - 12 + 7 + fdiff ));
        drone->setPitch(dronePitch);
        auto pmix = std::make_shared<PanningMixer>(pan);
        pmix->children.insert(drone);
        dmix->addInput( 0.3, pmix );
    }

    auto dlpf = std::make_shared<LPFBiquad>();
    dlpf->setFreq(2000);
    dlpf->setResonance(0.1);
    dlpf->setInput(dmix);

    auto dlpfmod = std::make_shared<LFOSin>( 1.27 );
    std::weak_ptr<LPFBiquad> wdlpf = dlpf;
    auto dlpfmodb = std::make_shared<ModulatorBinder>(dlpfmod,
                                                      [wdlpf](double v)
                                                          {
                                                              wdlpf.lock()->setFreq( 1700 + 300 * v );
                                                          }
        );
    dlpf->children.insert(dlpfmodb);
    auto droneEnv = std::make_shared<ADSRHeldForTimeEnv>( 0.1, 0.05, 1.0, 1.0, 30, 1.0 );
    auto droneFinal = std::make_shared<VCA>( dlpf, droneEnv );

    auto mix = std::make_shared<WeightedMixer>();
    float scale = 0.5;
    mix->addInput( scale, p );
    mix->addInput( scale * 0.6, del );
    mix->addInput( scale * 0.4, droneFinal );
    
    Renderer r(mix);
    r.writeToFile( "disquiet0413.wav", 32 );
    return 0;
}

int main( int argc, char **argv )
{
    runDisquiet0413();
    if( steppableCount != 0 )       std::cout << "*ERROR* you set up a cycle somewhere. Probably a modulator" << std::endl;
    return 0;
}
