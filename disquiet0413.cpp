#include <iostream>
#include <iomanip>
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
};

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

struct Env : public Steppable
{
    virtual double amplitude() = 0;
};

struct ConstantTimedEnv : public Env {
    double time, amp;
    double currtime;
    ConstantTimedEnv( double time, double amp ) : time(time), amp(amp), currtime(0) {
    }
    virtual double amplitude() override { return amp; }
    virtual bool isActive() override { return currtime < time; };
    virtual void step() override { currtime += stime; }
};

struct Note : public Steppable
{
    std::shared_ptr<Osc> source;
    std::shared_ptr<Env> env;
    Note( std::shared_ptr<Pitch> ipitch,
          std::shared_ptr<Osc> isource,
          std::shared_ptr<Env> ienvelope ) {
        source = isource;
        source->setPitch(ipitch);
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
    
int main( int arcgc, char **argv )
{
    std::cout << "Disquiet 0413" << std::endl;
}
