#define GAMMA_H_INC_ALL         // define this to include all header files
#define GAMMA_H_NO_IO           // define this to avoid bringing AudioIO from Gamma

#include "Gamma/Gamma.h"
#include "al/app/al_App.hpp"
#include "al/app/al_AudioApp.hpp"
#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Shapes.hpp"
//#include "al/core.hpp"
#include "al/sound/al_Speaker.hpp"
#include "al/ui/al_Parameter.hpp"
#include "al/ui/al_ParameterServer.hpp"
#include "Gamma/SamplePlayer.h"
#include "Gamma/Noise.h"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_HtmlInterfaceServer.hpp"

#include "al_ext/spatialaudio/al_Decorrelation.hpp"

#include "al/graphics/al_Font.hpp"

#include <atomic>
#include <vector>

#define SAMPLE_RATE 44100
//#define BLOCK_SIZE (2048)
// 1024 block size seems to fix issue where it sounded like samples were being dropped.
#define BLOCK_SIZE (1024)
#define MAX_DELAY 44100

#define NUM_SOURCES 5

using namespace al;
using namespace std;


//osc::Send sender(9011, "127.0.0.1");
//ParameterServer paramServer("127.0.0.1",8080);
//Parameter srcAzimuth("srcAzimuth","",-0.5,"",-1.0*M_PI,M_PI);
//ParameterBool updatePanner("updatePanner","",0.0);

float radius = 5.0;

ParameterBool sampleWise("sampleWise","",1.0);
ParameterBool useDelay("useDelay","", 0.0);

Parameter masterGain("masterGain","",0.5,"",0.0,1.0);
ParameterBool soundOn("soundOn","",0.0);

PresetHandler presets("data/presets");
PresetHandler srcPresets("data/srcPresets");

SearchPaths searchpaths;

vector<string> files;//{"count.wav","lowBoys.wav","midiPiano.wav","pianoA.wav","pianoB.wav","pianoC.wav","pianoD.wav","pizzPhase.wav","hornPhase.wav"};

vector<string> posUpdateNames{"off", "trajectory", "moving"};

vector<string> panningMethodNames{"VBAP","SpeakerSkirt","Snap to Source Width", "Snap To Nearest Speaker"};

Parameter maxDelay("maxDelay","",0.0,"",0.0,1.0);

//ParameterBool resetSamples("resetSamples","",0.0);
Trigger resetSamples("resetSamples","","");

ParameterMenu setAllPosUpdate("setAllPosUpdate","",0,"");
ParameterMenu setAllSoundFileIdx("setAllSoundFileIdx","",0,"");
ParameterBool setAllEnabled("setAllEnabled","",0.0);
ParameterMenu setAllPanMethod("setAllPanMethod","",0,"");

//ParameterBool setPiano("setPiano","",0.0);
//ParameterBool setMidiPiano("setMidiPiano","",0.0);
Trigger setPiano("setPiano","","");
Trigger setMidiPiano("setMidiPiano","","");


Parameter setAllAzimuth("setAllAzimuth","",2.9,"",-1.0*M_PI,M_PI);
Parameter azimuthSpread("azimuthSpread","",0.f,"",0.0,2.0*M_PI);

ParameterBool setAllRatesToOne("setAllRatesToOne","",0.0);

Parameter setPlayerPhase("setPlayerPhase","",0.0,"",0.0,44100.0);
//ParameterBool triggerAllRamps("triggerAllRamps","",0.0);
//ParameterBool setAllStartAzi("setAllStartAzi","",0.0);
//ParameterBool setAllEndAzi("setAllEndAzi","",0.0);
Trigger triggerAllRamps("triggerAllRamps","","");
Trigger setAllStartAzi("setAllStartAzi","","");
Trigger setAllEndAzi("setAllEndAzi","","");


Parameter setAllDurations("setAllDurations","",0.0,"",0.0,10.f);


ParameterBool combineAllChannels("combineAllChannels","",0.0);
//HtmlInterfaceServer interfaceServer("/Users/primary1/Documents/code/allolibCode/projects/interface.js");

Parameter setMorphTime("setMorphTime","",0.0,"",0.0, 10.0);
Parameter recallPreset("recallPreset","",0.0,"",0.0, 30.0);



ParameterMenu speakerDensity("speakerDensity","",0,"");
vector<string> spkDensityMenuNames{"All", "Skip 1", "Skip 2", "Skip 3", "Skip 4", "Skip 5"};


ParameterBool decorrelate("decorrelate","",0.0);
ParameterInt sourcesToDecorrelate("sourcesToDecorrelate","",2,"",1,2);
ParameterMenu decorrelationMethod("decorrelationMethod","",0,"");
vector<string> decorMethodMenuNames{"Kendall", "Zotter"};
ParameterBool configureDecorrelation("configureDecorrelation","",0.0);

Parameter deltaFreq("deltaFreq","",20.0,"", 0.0, 50.0);
Parameter maxFreqDev("maxFreqDev","",10.0,"", 0.0, 50.0);
Parameter maxTau("maxTau","",1.0,"", 0.0, 10.0);
Parameter startPhase("startPhase","",0.0,"", 0.0, 10.0);
Parameter phaseDev("phaseDev","",0.0,"", 0.0, 10.0);
Trigger updateDecorrelation("updateDecorrelation","","");

ParameterBool drawLabels("drawLabels","",1.0);

int highestChannel = 0;

mutex enabledSpeakersLock;

struct Ramp {

    unsigned int startSample, endSample;
    bool done = false;
    bool running = false;
    bool trigger = false;
    float rampStartAzimuth = 0.0f;
    float rampEndAzimuth = 0.0f;
    float rampDuration = 0.0f;

    Ramp(){


       // rampBundle  << rampStartAzimuth << rampEndAzimuth << rampDuration;

//        triggerRamp.registerChangeCallback([&](float val){
//            if(val == 1.f){ // is this correct way to check?
//                //cout << "Ramp triggered" << endl;
//                trigger = true;
//            }
//        });
    }

    void set(float startAzi, float endAzi, float dur){
//        rampStartAzimuth.set(startAzi);
//        rampEndAzimuth.set(endAzi);
//        rampDuration.set(dur);
        rampStartAzimuth = startAzi;
        rampEndAzimuth = endAzi;
        rampDuration = dur;
    }


    void start(unsigned int startSamp){
        startSample = startSamp;
//        endSample = startSamp +  rampDuration.get() *SAMPLE_RATE;
                endSample = startSamp +  rampDuration *SAMPLE_RATE;
        running = true;
        done = false;
    }

    void triggerRamp(){
        trigger = true;
    }

    float next(unsigned int sampleNum){

        if(trigger){
            trigger = false;
            start(sampleNum);
        }

        if(!done && !running){
//            return rampStartAzimuth.get();
                        return rampStartAzimuth;
        }else if(!done && running){

            if(sampleNum > endSample){
                sampleNum = endSample;
                done = true;
                running = false;
            }
//            float val = (((sampleNum - startSample) * (rampEndAzimuth.get() - rampStartAzimuth.get())) / (endSample - startSample)) + rampStartAzimuth.get();
            float val = (((sampleNum - startSample) * (rampEndAzimuth - rampStartAzimuth)) / (endSample - startSample)) + rampStartAzimuth;

            return val;
        } else {
            return rampEndAzimuth;
        }
    }
};

void initPanner();

void wrapValues(float &val){
    while(val > M_PI){
        val -= M_2PI;
    }
    while(val < -1.f*M_PI){
        val += M_2PI;
    }
}

class VirtualSource {
public:

    ParameterBool enabled{"enabled","",0.0};
    ParameterMenu positionUpdate{"positionUpdate","",0,""};
    Parameter sourceGain{"sourceGain","",0.5,"",0.0,1.0};
    Parameter aziInRad{"aziInRad","",2.9,"",-1.0*M_PI,M_PI};
    gam::SamplePlayer<> samplePlayer;
    gam::NoisePink<> noise;
    gam::Sine<> osc;
    Parameter oscFreq{"oscFreq","",440.0,"",0.0,2000.0f};
    //int sound = 0;
    //Parameter fileIdx{"fileIdx","",2.0,"",0.0,3.0};
    ParameterBundle vsBundle{"vsBundle"};
    float buffer[BLOCK_SIZE];
//    Parameter angularFreq {"angularFreq","",1.f,"",-1000.f,1000.f};
    Parameter angularFreq {"angularFreq"};//Radians per second
    Parameter angFreqCycles {"angFreqCycles", "",1.f,"",-1000.f,1000.f};

    Parameter samplePlayerRate {"samplePlayerRate","",1.f,"",1.f,1.5f};
    ParameterMenu fileMenu{"fileMenu","",0,""};
    Ramp sourceRamp;
    Trigger triggerRamp{"triggerRamp","",""};

    Parameter rampStartAzimuth{"rampStartAzimuth","",-0.5,"",-1.0*M_PI,M_PI};
    Parameter rampEndAzimuth{"rampEndAzimuth","",0.5,"",-1.0*M_PI,M_PI};
    Parameter rampDuration{"rampDuration", "",1.0,"",0.0,10.0};
    ParameterMenu sourceSound{"sourceSound","",0,""};

    int previousSamp = 0;

    Parameter sourceWidth{"sourceWidth","", M_PI/8.0f, "", 0.0f,M_PI};
    ParameterMenu panMethod{"panMethod","",0,""};

    ParameterBundle rampBundle{"rampBundle"};

    VirtualSource(){



        angularFreq.set(angFreqCycles.get()*M_2PI);

        osc.freq(oscFreq.get());

        panMethod.setElements(panningMethodNames);
        sourceSound.setElements({"SoundFile","Noise","Sine"});
        positionUpdate.setElements(posUpdateNames);
        fileMenu.setElements(files);
        samplePlayer.load("src/sounds/count.wav");
        samplePlayerRate.set(1.0 + (.002 * vsBundle.bundleIndex()));
        samplePlayer.rate(samplePlayerRate.get());

        sourceRamp.rampStartAzimuth = rampStartAzimuth.get();
        sourceRamp.rampEndAzimuth = rampEndAzimuth.get();
        sourceRamp.rampDuration = rampDuration.get();

        angFreqCycles.registerChangeCallback([&](float val){
            angularFreq.set(val*M_2PI);

        });

        oscFreq.registerChangeCallback([&](float val){
           osc.freq(val);
        });

        aziInRad.setProcessingCallback([&](float val){
            wrapValues(val);
            return val;
        });

        samplePlayerRate.registerChangeCallback([&](float val){
           samplePlayer.rate(val);
        });

        fileMenu.registerChangeCallback([&](float val){
            //cout << "FileMenu val: " << val << endl;
            samplePlayer.load(searchpaths.find(files[val]).filepath().c_str());
        });

//        sourceRamp.triggerRamp.registerChangeCallback([&](float val){
//            if(val == 1.f){ // is this correct way to check?
//                //cout << "Ramp triggered" << endl;
//                sourceRamp.trigger = true;
//                samplePlayer.reset();
//            }
//        });

        triggerRamp.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                //cout << "Ramp triggered" << endl;
//                sourceRamp.trigger = true;
                sourceRamp.triggerRamp();
                samplePlayer.reset();
            }
        });

        rampStartAzimuth.registerChangeCallback([&](float val){
                sourceRamp.rampStartAzimuth = val;
        });

        rampEndAzimuth.registerChangeCallback([&](float val){
                sourceRamp.rampEndAzimuth = val;
        });

        rampDuration.registerChangeCallback([&](float val){
                sourceRamp.rampDuration = val;
        });

        //rampBundle.
        //rampBundle << triggerRamp << rampStartAzimuth << rampEndAzimuth << rampDuration;
//        vsBundle << enabled << sourceGain << aziInRad << positionUpdate << fileMenu << samplePlayerRate << triggerRamp << sourceRamp.rampStartAzimuth << sourceRamp.rampEndAzimuth << sourceRamp.rampDuration << angularFreq;
        vsBundle << enabled << panMethod << positionUpdate << sourceSound <<  fileMenu << sourceGain << aziInRad   << samplePlayerRate  << angularFreq << angFreqCycles << oscFreq  << sourceWidth ;

        srcPresets << vsBundle;
    }

    void updatePosition(unsigned int sampleNumber){

        switch ((int)positionUpdate.get()) {
        case 0:
            break;
        case 1:
            aziInRad.set(sourceRamp.next(sampleNumber));
            break;
        case 2: {
            float aziDelta = angularFreq.get()*(sampleNumber - previousSamp)/SAMPLE_RATE;
            aziInRad.set(aziInRad.get()+aziDelta);
            previousSamp = sampleNumber;
            break;
        }
        default:
            break;
        }
    }

    float getSample(){
        switch ((int)sourceSound.get() ) {
        case 0:
            if(samplePlayer.done()){
                samplePlayer.reset();
            }
            return sourceGain.get() * samplePlayer();
            break;

        case 1:
            return sourceGain.get() * noise();
            break;
        case 2:
            return sourceGain.get() * osc() * 0.2;
        default:
            return 0.0;
            break;
        }
    }
    
    void getBuffer(float *buffer){
        for(int i = 0; i < BLOCK_SIZE; i++){
            buffer[i] = getSample();
        }
    }

//    float getSample(){
//        return getSampleForSource();
//    }

    float getSamplePlayerPhase(){
        return samplePlayer.pos()/samplePlayer.frames();
    }
};

//VirtualSource vs;
vector<VirtualSource*> sources;

class SpeakerV: public Speaker {
public:

    ParameterBool *enabled;
    std::string oscTag;
    std::string deviceChannelString;
    float aziInRad;

    int delay;
    void *buffer;
    int bufferSize;
    int readPos,writePos;

    bool isPhantom = false;

    //Parameter spkSkirtWidth{"spkSkirtWidth","", M_PI/8.0f, "", 0.0f,M_PI};

    SpeakerV(int chan, float az=0.f, float el=0.f, int gr=0, float rad=1.f, float ga=1.f, int del = 0){
        delay = del;
        readPos = 0;
        writePos = 0;

        setDelay(maxDelay.get()*SAMPLE_RATE);
        bufferSize = 44100*2;//MAKE WORK WITH MAX DELAY
        buffer = calloc(bufferSize,sizeof(float));
        deviceChannel = chan;
        azimuth= az;
        elevation = el;
        group = gr;
        radius = rad;
        gain = ga;
        aziInRad = toRad(az);
        deviceChannelString = std::to_string(deviceChannel);
        oscTag = "speaker"+ deviceChannelString + "/enabled";

        enabled = new ParameterBool(oscTag,"",1.0);
        enabled->registerChangeCallback([&](bool b){
            initPanner(); //CALLED MULTIPLE TIMES WHEN USING PRESETS
        });

        //paramServer.registerParameter(*enabled);
    }

    void setDelay(int delayInSamps){
        delay = rand() % static_cast<int>(delayInSamps + 1);
    }

    float read(){
        if(readPos >= bufferSize){
            readPos = 0;
        }
        float *b = (float*)buffer;
        float val = b[readPos];
        readPos++;
        return val;
    }

    void write(float samp){
        if(writePos >= bufferSize){
            writePos -= bufferSize;
        }

        int writeDelay = writePos + delay;
        if(writeDelay >= bufferSize){
            writeDelay -= bufferSize;
        }

        float *b = (float*)buffer;
        b[writeDelay] = samp;
        writePos++;
    }
};

std::vector<SpeakerV> speakers;
std::vector<SpeakerV*> enabledSpeakers;
Mat<2,double> matrix;

bool speakerSort(SpeakerV const *first, SpeakerV const *second){
    return first->azimuth < second->azimuth;
}

void initPanner(){

    enabledSpeakersLock.lock();

    enabledSpeakers.clear();
    for(int i = 0; i < speakers.size(); i ++){
        if(speakers[i].enabled->get() > 0.5){
            enabledSpeakers.push_back(&speakers[i]);
        }
    }
    std::sort(enabledSpeakers.begin(),enabledSpeakers.end(),&speakerSort);

    enabledSpeakersLock.unlock();
    //cout <<"panner init" << endl;
}

//class MyApp : public App
struct MyApp : public App
{
public:
    Mesh mSpeakerMesh;
    vector<Mesh> mVec;
    vector<int>  sChannels;
    SpeakerLayout speakerLayout;
    atomic<float> *mPeaks {nullptr};
    float speedMult = 0.03f;
    Vec3d srcpos {0.0,0.0,0.0};
    ControlGUI parameterGUI;
//    ControlGUI speakerGUI;
    ParameterBundle xsetAllBundle{"xsetAllBundle"};


    

    //Size of the decorrelation filter. See Kendall p. 75
    //How does this translate to the duration of the impulse response?
    Decorrelation decorrelation{BLOCK_SIZE*2}; //Check to see if this is correct

    float mMaxjump{M_PI};
    map<uint32_t, vector<uint32_t>> routingMap;

    Font font;
    Mesh fontMesh;

    MyApp()
    {

        //cout << "Constructor" << endl;
        searchpaths.addAppPaths();
        searchpaths.addRelativePath("src/sounds");
        //searchpaths.print();

        FileList fl = itemListInDir("src/sounds");
        for(int i = 0; i < fl.count(); i++){
            string fPath = fl[i].filepath();
            fPath = fPath.substr(fPath.find_last_of("/\\")+1);
            //cout << fPath << endl;
            files.push_back(fPath);
        }

        //cout << "Path of Count " << searchpaths.find("count.wav").filepath().c_str() << endl;
//        parameterGUI << soundOn << resetSamples << sampleWise << useDelay << masterGain << maxDelay << combineAllChannels << decorrelate << decorrelationMethod << deltaFreq << maxFreqDev << maxTau << startPhase << phaseDev << updateDecorrelation << speakerDensity << drawLabels;

        parameterGUI << soundOn << masterGain << resetSamples << sampleWise  << combineAllChannels << decorrelate << sourcesToDecorrelate << decorrelationMethod << deltaFreq << maxFreqDev << maxTau << startPhase << phaseDev << updateDecorrelation << speakerDensity << drawLabels;

        parameterGUI << srcPresets;

        xsetAllBundle << setAllEnabled << setAllPanMethod << setAllPosUpdate << setAllSoundFileIdx <<setAllAzimuth << azimuthSpread << setAllRatesToOne << setPlayerPhase << triggerAllRamps << setAllStartAzi << setAllEndAzi << setAllDurations << setPiano << setMidiPiano;
        parameterGUI << xsetAllBundle;

        for(int i = 0; i < NUM_SOURCES; i++){
            auto *newVS = new VirtualSource; // This memory is not freed and it should be...
            sources.push_back(newVS);
            // Register its parameter bundle with the ControlGUI
            parameterGUI << newVS->vsBundle;

            parameterServer() << newVS->vsBundle;
        }

        //parameterGUI.

        //parameterGUI << presets;

        parameterServer() << soundOn << resetSamples << sampleWise << useDelay << masterGain << maxDelay << xsetAllBundle << setMorphTime << recallPreset << combineAllChannels << decorrelate << decorrelationMethod << speakerDensity << drawLabels;

        setAllPanMethod.setElements(panningMethodNames);
        setAllPosUpdate.setElements(posUpdateNames);
        setAllSoundFileIdx.setElements(files);

        speakerDensity.setElements(spkDensityMenuNames);

        decorrelationMethod.setElements(decorMethodMenuNames);

        decorrelationMethod.registerChangeCallback([&](float val){
            configureDecorrelation.set(1.0);
        });

        updateDecorrelation.registerChangeCallback([&](float val){
            configureDecorrelation.set(1.0);
        });

        setMorphTime.registerChangeCallback([&](float val){
            srcPresets.setMorphTime(val);
        });

        recallPreset.registerChangeCallback([&](float val){
            srcPresets.recallPreset(val);
        });

        setPlayerPhase.registerChangeCallback([&](float val){
            VirtualSource *firstSource = sources[0];
            int firstPos = firstSource->samplePlayer.pos();
            for(VirtualSource *v: sources){
                v->samplePlayer.reset();
                int idx = v->vsBundle.bundleIndex();
                int newPos = firstPos + (val*idx);
                if(newPos >= v->samplePlayer.frames()){
                    newPos -= v->samplePlayer.frames();
                }
                v->samplePlayer.pos(newPos);
            }
        });

        setAllRatesToOne.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                if(val == 1.f){
                    v->samplePlayer.rate(1.0);
                }else{
                    v->samplePlayer.rate(v->samplePlayerRate.get());
                }
            }
        });

        azimuthSpread.registerChangeCallback([&](float val){
            azimuthSpread.setNoCalls(val);
            setAllAzimuth.set(setAllAzimuth.get());
        });

        setAllAzimuth.registerChangeCallback([&](float val){
            if(sources.size() > 1){
                float aziInc = azimuthSpread.get()/(sources.size()-1);
                float startAzi = val - (azimuthSpread.get()/2.0);
                for(VirtualSource *v: sources){
                    int idx = v->vsBundle.bundleIndex();
                    float newAzi = startAzi + (aziInc*idx);
                    wrapValues(newAzi);
                    v->aziInRad.set(newAzi);
                }
            }else{
                VirtualSource *v = sources[0];
                v->aziInRad.set(val);
            }
        });

        setAllEnabled.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                v->enabled.set(val);
            }
        });

        setAllPanMethod.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                v->panMethod.set(val);
            }
        });
        
        setPiano.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                switch (v->vsBundle.bundleIndex()) {
                    case 0:
                        v->samplePlayer.load("src/sounds/pianoA.wav");
                        v->enabled.set(1.f);
                        break;
                    case 1:
                        v->samplePlayer.load("src/sounds/pianoB.wav");
                        v->enabled.set(1.f);
                        break;
                    case 2:
                        v->samplePlayer.load("src/sounds/pianoC.wav");
                        v->enabled.set(1.f);
                        break;
                    case 3:
                        v->samplePlayer.load("src/sounds/pianoD.wav");
                        v->enabled.set(1.f);
                        break;
                    default:
                        v->enabled.set(0.f);
                        break;
                }
            }
        });

         setMidiPiano.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                switch (v->vsBundle.bundleIndex()) {
                    case 0:
                        v->samplePlayer.load("src/sounds/midiPiano.wav");
                        v->enabled.set(1.f);
                        break;
                    case 1:
                        v->samplePlayer.load("src/sounds/midiPiano.wav");
                        v->enabled.set(1.f);
                        break;
                    case 2:
                        v->samplePlayer.load("src/sounds/midiPiano.wav");
                        v->enabled.set(1.f);
                        break;
                    case 3:
                        v->samplePlayer.load("src/sounds/midiPiano.wav");
                        v->enabled.set(1.f);
                        break;
                    default:
                        v->enabled.set(0.f);
                        break;
                }
            }
        });

        setAllPosUpdate.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                v->positionUpdate.set(val);
            }
        });

        setAllSoundFileIdx.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
                v->fileMenu.set(val);
            }
        });

        triggerAllRamps.registerChangeCallback([&](float val){
            if(val == 1.f){
                for(VirtualSource *v: sources){
//                    v->sourceRamp.triggerRamp.set(1.f);
                    v->triggerRamp.set(1.f);
                }
            }
        });

        setAllStartAzi.registerChangeCallback([&](float val){
            if(val == 1.f){
                for(VirtualSource *v: sources){
                    v->rampStartAzimuth.set(v->aziInRad);
                }
            }
        });

        setAllEndAzi.registerChangeCallback([&](float val){
            if(val == 1.f){
                for(VirtualSource *v: sources){
                    v->rampEndAzimuth.set(v->aziInRad);
                }
            }
        });

        setAllDurations.registerChangeCallback([&](float val){

            for(VirtualSource *v: sources){
                v->rampDuration.set(val);
            }
        });

//        updatePanner.registerChangeCallback([&](float val){
//            if(val == 1.f){ // is this correct way to check?
//                cout << "panner Updated" << endl;
//                initPanner();
//            }
//        });

        maxDelay.registerChangeCallback([&](float val){
            int delSamps = val*SAMPLE_RATE;
            for(SpeakerV &v:speakers){
               v.setDelay(delSamps);
            }
        });

        resetSamples.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                for(VirtualSource *v: sources){
                    v->samplePlayer.reset();
                }
            }
        });

        speakerDensity.registerChangeCallback([&](float val){

            //cout << "Menu item chosen: " << val << endl;
            for(int i = 0; i < speakers.size(); i++){
                SpeakerV v = speakers[i];
                if(v.deviceChannel != -1){
                    int tempVal = v.deviceChannel % ((int)val+1);
                    //cout << "TempVal " << tempVal << endl;
                    if(tempVal == 0){
                        v.enabled->set(1.0f);
                    }else {
                        v.enabled->set(0.0f);
                    }
                }
            }
        });


//        std::vector<Parameter *> paramsList = parameterServer().parameters();
//        cout << "paramList size: " << paramsList.size() << endl;
//        for(int i = 0; i < paramsList.size(); i++){
//            cout << "in stringparams" << endl;
//            cout << paramsList[i]->getFullAddress() << endl;
//        }

        parameterServer().sendAllParameters("127.0.0.1", 9011);
    }

    void createMatrix(Vec3d left, Vec3d right){
        matrix.set(left.x,left.y,right.x,right.y);
    }

    Vec3d ambiSphericalToOGLCart(float azimuth, float radius){
        Vec3d ambiSpherical;
        float elevation = 0.0;

        //find xyz in cart audio coords
        float x = radius * cos(elevation) * cos(azimuth);
        float y = radius * cos(elevation) * sin(azimuth);
        float z = radius * sin(elevation);

        //convert to open_gl cart coords
        ambiSpherical[0] = -y;
        ambiSpherical[1] = z;
        ambiSpherical[2] = -x;

        return ambiSpherical;
    }

    void openGLCartToAmbiCart(Vec3f &vec){
        Vec3f tempVec = vec;
        vec[0] = tempVec.z*-1.0;
        vec[1] = tempVec.x*-1.0;
        vec[2] = tempVec.y;
    }

    //TODO: io not used here
    Vec3d calcGains(AudioIOData &io, const float &srcAzi, int &speakerChan1, int &speakerChan2){

        Vec3f ambiCartSrcPos = ambiSphericalToOGLCart(srcAzi,radius);
        openGLCartToAmbiCart(ambiCartSrcPos);
        //std::sort(enabledSpeakers.begin(),enabledSpeakers.end(),&speakerSort);
        Vec3d gains(0.,0.,0.);
        float speakSrcAngle,linearDistance;

        if(enabledSpeakersLock.try_lock()){ //TODO: handle differently. enabled speakers is cleared by initPanner() and needs to be locked when updating
            //check if source is beyond the first or last speaker
            if(srcAzi < enabledSpeakers[0]->aziInRad){
                speakerChan1 = enabledSpeakers[0]->deviceChannel;
                speakerChan2 = enabledSpeakers[0+1]->deviceChannel;
                speakSrcAngle = fabsf(enabledSpeakers[0]->aziInRad - srcAzi);
                gains.x = 1.f / (radius * (M_PI - speakSrcAngle));

            } else if(srcAzi > enabledSpeakers[enabledSpeakers.size()-1]->aziInRad){
                speakerChan1 = enabledSpeakers[enabledSpeakers.size()-2]->deviceChannel;//set to speaker before last
                speakerChan2 = enabledSpeakers[enabledSpeakers.size()-1]->deviceChannel;
                speakSrcAngle = fabsf(enabledSpeakers[enabledSpeakers.size()-1]->aziInRad - srcAzi);
                linearDistance = 2.0*radius*cos((M_PI - speakSrcAngle)/2.0);
                gains.y = 1.f / (radius * (M_PI - speakSrcAngle));

            } else{//Source between first and last speakers
                for(int i = 0; i < enabledSpeakers.size()-1; i++){
                    speakerChan1 = enabledSpeakers[i]->deviceChannel;
                    speakerChan2 = enabledSpeakers[i+1]->deviceChannel;
                    if(srcAzi == enabledSpeakers[i]->aziInRad ){
                        gains.x = 1.0;
                        break;
                    }else if(srcAzi > enabledSpeakers[i]->aziInRad && srcAzi < enabledSpeakers[i+1]->aziInRad){
                        createMatrix(enabledSpeakers[i]->vec(),enabledSpeakers[i+1]->vec());
                        invert(matrix);
                        for (unsigned i = 0; i < 2; i++){
                            for (unsigned j = 0; j < 2; j++){
                                gains[i] += ambiCartSrcPos[j] * matrix(j,i);
                            }
                        }
                        gains = gains.normalize();
                        break;
                    } else if(srcAzi == enabledSpeakers[i+1]->aziInRad){
                        gains.y = 1.0;
                        break;
                    }
                }
            }

            enabledSpeakersLock.unlock();

        }else{
            speakerChan1 = speakerChan2 = -1;
        }
        return gains;
    }

    //Returns the magnitude of the angle difference from 0 - Pi
    float getAbsAngleDiff(float const &angle1, float const &angle2){
        float diff = angle1 - angle2;
        diff += (diff > M_PI) ? -1.0f*M_2PI : (diff < -1.0f*M_PI) ? M_2PI : 0.0f;
        return fabs(diff);
    }


    float calcSpeakerSkirtGains(float srcAzi, float spkSkirtWidth, float skirtWidthMax, float spkAzi){
        float gain = 0.0f;
        float distanceToSpeaker = getAbsAngleDiff(srcAzi, spkAzi);
        if(distanceToSpeaker <= spkSkirtWidth/2.0f){
            float p = ((2.0f*distanceToSpeaker)/(spkSkirtWidth/2.0f))-1.0f;
            gain = cos((M_PI*(p+1))/4.0f);
            return gain;
        }else{
            return 0.0f;
        }
    }

    void setOutput(AudioIOData &io, int channel, int frame, float sample){
        if(channel != -1){
            io.out(channel,frame) += sample*masterGain.get();
        }
    }

    void renderBufferDelaySpeakers(AudioIOData &io,const float &srcAzi, const float *buffer){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io,srcAzi, speakerChan1, speakerChan2);

        for(int i = 0; i < enabledSpeakers.size(); i++){
            SpeakerV *s = enabledSpeakers[i];
            for(int j = 0; j < io.framesPerBuffer();j++){
                if(s->deviceChannel == speakerChan1){
                    s->write(buffer[j]*gains[0]);
                }else if(s->deviceChannel == speakerChan2){
                    s->write(buffer[j]*gains[1]);
                }else{
                    s->write(0.0);
                }
                setOutput(io,s->deviceChannel,j,s->read());
            }
        }
    }

    void renderSampleDelaySpeakers(AudioIOData &io,const float &srcAzi, const float &sample){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io,srcAzi, speakerChan1, speakerChan2);

        //TODO:
        for(int i = 0; i < enabledSpeakers.size(); i++){
            SpeakerV *s = enabledSpeakers[i];
            if(s->deviceChannel == speakerChan1){
                s->write(sample*gains[0]);
            }else if(s->deviceChannel == speakerChan2){
                s->write(sample*gains[1]);
            }else{
                s->write(0.0);
            }
            setOutput(io,s->deviceChannel,io.frame(),s->read());
        }
    }

    void renderBuffer(AudioIOData &io,const float &srcAzi, const float *buffer){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io,srcAzi, speakerChan1, speakerChan2);
        for(int i = 0; i < io.framesPerBuffer(); i++){
            setOutput(io,speakerChan1,i,buffer[i]*gains[0]);
            setOutput(io,speakerChan2,i,buffer[i]*gains[1]);
        }
    }

//    void renderSample(AudioIOData &io, const float &srcAzi, const float &sample, int pMethod ){
    void renderSample(AudioIOData &io, VirtualSource  *vs, bool const &decorrelateBlock){

        int outputBufferOffset = (highestChannel+1)*vs->vsBundle.bundleIndex();

        if(vs->panMethod.get() == 0){ // VBAP
            int speakerChan1, speakerChan2;
            Vec3d gains = calcGains(io,vs->aziInRad, speakerChan1, speakerChan2);
            float sampleOut1, sampleOut2;
            if(decorrelateBlock){

                if(speakerChan1 != -1){
                     sampleOut1 = decorrelation.getOutputBuffer(speakerChan1+outputBufferOffset)[io.frame()];
                    setOutput(io,speakerChan1,io.frame(),sampleOut1 * gains[0]);
                }
                if(speakerChan2 != -1){
                     sampleOut2 = decorrelation.getOutputBuffer(speakerChan2+outputBufferOffset)[io.frame()];
                    setOutput(io,speakerChan2,io.frame(),sampleOut2 * gains[1]);
                }
            } else{ // don't decorrelate
                float sample = vs->getSample();
                setOutput(io,speakerChan1,io.frame(),sample * gains[0]);
                setOutput(io,speakerChan2,io.frame(),sample * gains[1]);
            }


        }else if(vs->panMethod.get()==1){ //Skirt

            float sample = 0.0f;
            if(!decorrelateBlock){
                 sample = vs->getSample();
            }
            for (int i = 0; i < speakers.size(); i++){
                if(!speakers[i].isPhantom && speakers[i].enabled->get()){
                    int speakerChannel = speakers[i].deviceChannel;
                    float gain = calcSpeakerSkirtGains(vs->aziInRad,vs->sourceWidth,vs->sourceWidth.max(),speakers[i].aziInRad);

                    if(decorrelateBlock){
                        sample = decorrelation.getOutputBuffer(speakerChannel + outputBufferOffset)[io.frame()];
                        setOutput(io,speakerChannel,io.frame(),sample * gain);

                    }else{
                        setOutput(io,speakerChannel,io.frame(),sample * gain);
                    }
                }
            }
        }else if(vs->panMethod.get() == 2){ // Snap Source Width
            float sample = 0.0f;
            if(!decorrelateBlock){
                 sample = vs->getSample();
            }
            for (int i = 0; i < speakers.size(); i++){
                if(!speakers[i].isPhantom && speakers[i].enabled->get()){
                    int speakerChannel = speakers[i].deviceChannel;
                    float angleDiff = getAbsAngleDiff(vs->aziInRad,speakers[i].aziInRad);
                    if(angleDiff <= vs->sourceWidth/2.0f){
                        if(decorrelateBlock){
                             sample = decorrelation.getOutputBuffer(speakerChannel+ outputBufferOffset)[io.frame()];
                            setOutput(io,speakerChannel,io.frame(),sample);
                        }else{
                            setOutput(io,speakerChannel,io.frame(),sample);
                        }
                    }else{
                        setOutput(io,speakerChannel,io.frame(),0.0f);
                    }
                }
            }
        }else if(vs->panMethod.get() == 3){ //Snap to Nearest Speaker
            float smallestAngle = M_2PI;
            int smallestAngleSpkIdx = -1;
            for (int i = 0; i < speakers.size(); i++){
                if(!speakers[i].isPhantom && speakers[i].enabled->get()){

                    float angleDiff = getAbsAngleDiff(vs->aziInRad,speakers[i].aziInRad);

                    if(angleDiff < smallestAngle){
                        smallestAngle = angleDiff;
                        smallestAngleSpkIdx = i;
                    }
                    //setOutput(io,speakers[i].deviceChannel,io.frame(),0.0f);
                }
            }

            if(smallestAngleSpkIdx != -1){
                int speakerChannel = speakers[smallestAngleSpkIdx].deviceChannel;
                float sample = 0.0f;
                if(decorrelateBlock){
                    sample = decorrelation.getOutputBuffer(speakerChannel+ outputBufferOffset)[io.frame()];
                    setOutput(io,speakerChannel,io.frame(),sample);
                }else{
                    sample = vs->getSample();
                    setOutput(io,speakerChannel,io.frame(),sample);
                }
            }
        }
    }

    void onInit() override {

        //cout << "onInit()" << endl;

        float startingAngle = 170.0f;
//        float angleInc = 11.0f;
        float angleInc = 11.0f;

        float ang;
        for (int i = 0; i < 32; i++){
            int delay = rand() % static_cast<int>(MAX_DELAY + 1);
            ang = startingAngle - (angleInc*i);
            speakers.push_back(SpeakerV(i,ang,0.0,0,5.0,0,delay));
        }

        //-1 for phantom channels (can remove isPhantom and just check -1)
        SpeakerV s(-1, startingAngle+angleInc,0.0,0,5.0,0,0);
        s.isPhantom = true;
        speakers.push_back(s);

        SpeakerV p(-1, ang - angleInc,0.0,0,5.0,0,0);
        p.isPhantom = true;
        speakers.push_back(p);

//        SpeakerV p(-1, ang-10.0,0.0,0,5.0,0,0);
//        p.isPhantom = true;
//        speakers.push_back(p);
       // speakers.push_back(SpeakerV(-1, -100,0.0,0,5.0,0,0));

        initPanner();

        for(int i = 0; i < speakers.size(); i++){
            parameterGUI << speakers[i].enabled;
            presets << *speakers[i].enabled;
        }

//        int highestChannel = 0;
        for(SpeakerV s:speakers){
            if((int) s.deviceChannel > highestChannel){
                highestChannel = s.deviceChannel;
            }
        }

        audioIO().channelsOut(highestChannel + 1);
        audioIO().channelsOutDevice();

        cout << "Hightst Channel: " << highestChannel << endl;

        mPeaks = new atomic<float>[highestChannel + 1];

        addSphere(mSpeakerMesh, 1.0, 5, 5);
        mSpeakerMesh.primitive(Mesh::LINES);
        
//        map<uint32_t, vector<uint32_t>> routingMap = {
//            {0, {0, 1}, 1, {2, 3}, 2, {4, 5}, 3, {6, 7}, 4, {8, 9}, 5, {10, 11}}};

        //5 key values for now, one for each source
//        map<uint32_t, vector<uint32_t>> routingMap = {
//            {0, {0, 1}}, {1, {2, 3}}, {2, {4, 5}}, {3, {6, 7}}, {4, {8, 9}}};
//        map<uint32_t, vector<uint32_t>> routingMap;

        uint32_t key = 0;
        uint32_t val = 0;

        //TODO: this is for one source only, "ERROR convolution config failed" if > than 2 sources
        // MAXINP set to 64 in zita-convolver.h line 362

        for(int i = 0 ; i < sourcesToDecorrelate.get(); i++){
            vector<uint32_t> values;
            for(int j = 0; j < (highestChannel + 1); j++){
                values.push_back(val);
                val++;
            }
            routingMap.insert(std::pair<uint32_t,vector<uint32_t>>(key,values));

          key++;
        }
        for(auto it = routingMap.cbegin(); it != routingMap.cend(); ++it)
        {
            std::cout << it->first << " " << it->second[0] << endl;
            //std::cout << it->second << endl;
        }
        //cout << "Routing Map: " << routingMap.

        decorrelation.configure(audioIO().framesPerBuffer(), routingMap,
                                true, 1000, mMaxjump);

    }

    void onCreate() override {
        //cout << "onCreate()" << endl;
        nav().pos(0, 1, 20);
        parameterGUI.init();

        font.load("data/VeraMono.ttf", 28, 1024);
        font.alignCenter();
        font.write(fontMesh, "1", 0.2f);

        //speakerGUI.init();
        //initIMGUI();
        audioIO().start();
    }

//    void onExit() override {
//        shutdownIMGUI();
//    }

    void onAnimate( double dt) override {
        navControl().active(!parameterGUI.usingInput());
//        navControl().active(!parameterGUI.usingInput() && !speakerGUI.usingInput());
    }

    virtual void onSound(AudioIOData &io) override {

        static unsigned int t = 0;
        float srcBuffer[BLOCK_SIZE];
        float enabledSources = 0.0f;
        gam::Sync::master().spu(audioIO().fps());

        bool decorrelateBlock = decorrelate.get();

        if(soundOn.get()){
            
            if(decorrelateBlock){

                //Only reconfigure if method has changed
                if(configureDecorrelation.get()){
                    configureDecorrelation.set(0.0);
                    if(!decorrelationMethod.get()){
                        //Kendall Method
                        decorrelation.configure(audioIO().framesPerBuffer(), routingMap,
                                                true, 1000, mMaxjump);
                    }else {
                        //Zotter Method
                        decorrelation.configureDeterministic(audioIO().framesPerBuffer(), routingMap, true, 1000, deltaFreq, maxFreqDev, maxTau, startPhase, phaseDev);
                    }
                }

                for(VirtualSource *v: sources){
                    if(v->enabled){
                        auto inBuffer = decorrelation.getInputBuffer(v->vsBundle.bundleIndex());
                        for(int i = 0; i < BLOCK_SIZE; i++){
                            inBuffer[i] = v->getSample();
                        }
                    }
                    if(sourcesToDecorrelate.get() == v->vsBundle.bundleIndex()+1){
                         break;
                    }
                }
                decorrelation.processBuffer();
            }
            
            
            while (io()) {

                ++t;

                if(sampleWise.get()){
                        for(VirtualSource *v: sources){
                            if(v->enabled){
                                enabledSources += 1.0f;
                                v->updatePosition(t);
                                renderSample(io,v,decorrelateBlock);
                            }
                            if(decorrelateBlock){
                                if(sourcesToDecorrelate.get() == v->vsBundle.bundleIndex()+1){
                                     break;
                                }
                            }
                        }
                }
            }


            //TODO: Buffer based processing only uses VBAP
            if(!sampleWise.get()){
                if(decorrelateBlock){
                    for(VirtualSource *v: sources){
                        if(v->enabled){
                            enabledSources += 1.0f;
                            v->updatePosition(t);
                            int speakerChan1, speakerChan2;
                            Vec3d gains = calcGains(io,v->aziInRad.get(), speakerChan1, speakerChan2);

                            if(speakerChan1 != -1){
                                auto outputBuffer1 = decorrelation.getOutputBuffer(speakerChan1);
                                for(int i = 0; i < io.framesPerBuffer(); i++){
                                    setOutput(io,speakerChan1,i,outputBuffer1[i] * gains[0]);
                                }
                            }
                            if(speakerChan2 != -1){
                                auto outputBuffer2 = decorrelation.getOutputBuffer(speakerChan2);
                                for(int i = 0; i < io.framesPerBuffer(); i++){
                                    setOutput(io,speakerChan2,i,outputBuffer2[i] * gains[1]);
                                }
                            }
                        }
                        if(sourcesToDecorrelate.get() == v->vsBundle.bundleIndex()+1){
                             break;
                        }
                    }

                } else{

                    for(VirtualSource *v: sources){
                        if(v->enabled){
                            enabledSources += 1.0f;
                            v->updatePosition(t);
                            v->getBuffer(srcBuffer);
                            if(useDelay.get() == 1.f){
                                renderBufferDelaySpeakers(io,v->aziInRad.get(), srcBuffer);
                            }else{
                                renderBuffer(io,v->aziInRad.get(), srcBuffer);
                            }
                        }
                    }
                }
            }

            if(combineAllChannels.get()){
                float combineBuffer[BLOCK_SIZE];
                for(int i = 0; i < BLOCK_SIZE;i++){
                    combineBuffer[i] = 0.0f;
                }
                //combine all the channels into one buffer
                for (int speaker = 0; speaker < speakers.size(); speaker++) {
                    if(!speakers[speaker].isPhantom && speakers[speaker].enabled->get()){
                        int deviceChannel = speakers[speaker].deviceChannel;

                        for (int i = 0; i < io.framesPerBuffer(); i++) {
                            if(deviceChannel < io.channelsOut()) {
                                combineBuffer[i] += io.out(deviceChannel, i)/enabledSources;
                            }
                        }
                    }
                }

                //copy combined buffer to all channels
                for (int speaker = 0; speaker < speakers.size(); speaker++) {
                    if(!speakers[speaker].isPhantom && speakers[speaker].enabled->get()){
                        int deviceChannel = speakers[speaker].deviceChannel;

                        for (int i = 0; i < io.framesPerBuffer(); i++) {
                            if(deviceChannel < io.channelsOut()) {
                                io.out(deviceChannel,i) = combineBuffer[i]*masterGain.get();
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < speakers.size(); i++) {
            mPeaks[i].store(0);
        }
        for (int speaker = 0; speaker < speakers.size(); speaker++) {
            if(!speakers[speaker].isPhantom){
                float rms = 0;
                int deviceChannel = speakers[speaker].deviceChannel;

                for (int i = 0; i < io.framesPerBuffer(); i++) {
                    if(deviceChannel < io.channelsOut()) {
                        float sample = io.out(deviceChannel, i);
                        rms += sample * sample;
                    }
                }
                rms = sqrt(rms/io.framesPerBuffer());
                mPeaks[deviceChannel].store(rms);
            }
        }
    }

    virtual void onDraw(Graphics &g) override {

        g.clear(0);
        g.blending(true);
        g.blendModeAdd();

        g.pushMatrix();
        Mesh lineMesh;
        lineMesh.vertex(0.0,0.0, 10.0);
        lineMesh.vertex(0.0,0.0, -10.0);
        lineMesh.index(0);
        lineMesh.index(1);
        lineMesh.primitive(Mesh::LINES);
        g.color(1);
        g.draw(lineMesh);
        g.popMatrix();

        static int t = 0;
        t++;
        //Draw the sources
        for(VirtualSource *v: sources){
            Vec3d pos = ambiSphericalToOGLCart(v->aziInRad,radius);
            g.pushMatrix();
            g.translate(pos);


            if(drawLabels.get()){
                //Draw Source Index
                g.pushMatrix();
                g.color(1);
                g.translate(0.0,0.3,0.0);
                font.write(fontMesh,std::to_string(v->vsBundle.bundleIndex()).c_str(),0.2f);
                g.texture();
                font.tex.bind();
                g.draw(fontMesh);
                font.tex.unbind();
                g.popMatrix();
            }

            g.scale(0.3);
            g.color(0.4,0.4, 0.4, 0.5);
            g.draw(mSpeakerMesh);
            g.popMatrix();

            g.pushMatrix();
            //double sampPos = v->samplePlayer.pos();
            //double tPos = v->samplePlayer.posInInterval(sampPos);
//            if(t%100 == 0){
//                cout << v->getSamplePlayerPhase() << endl;
//            }
            g.translate(v->getSamplePlayerPhase(),2.0 + v->vsBundle.bundleIndex()*0.25,0.0);
            g.scale(0.3);
            g.draw(mSpeakerMesh);
            g.popMatrix();

            // Draw line
            Mesh lineMesh;
            lineMesh.vertex(0.0,0.0, 0.0);
            lineMesh.vertex(pos.x,0.0, pos.z);
            lineMesh.vertex(pos);
            lineMesh.index(0);
            lineMesh.index(1);
            lineMesh.index(1);
            lineMesh.index(2);
            lineMesh.index(2);
            lineMesh.index(0);
            lineMesh.primitive(Mesh::LINES);
            g.color(1);
            g.draw(lineMesh);
        }

        //Draw the speakers
        for(int i = 0; i < speakers.size(); ++i){
            int devChan = speakers[i].deviceChannel;
            g.pushMatrix();
            g.translate(speakers[i].vecGraphics());


            if(drawLabels.get()){
                //Draw Speaker Channel
                g.pushMatrix();
                g.translate(0.0,0.1,0.0);
                if(!speakers[i].isPhantom){
                    font.write(fontMesh,speakers[i].deviceChannelString.c_str(),0.2f);
                }else{
                    font.write(fontMesh,"P",0.2f);
                }
                g.texture();
                font.tex.bind();
                g.draw(fontMesh);
                font.tex.unbind();
                g.popMatrix();
            }

            float peak = 0.0;
            if(!speakers[i].isPhantom){
                peak = mPeaks[devChan].load();
            }
            g.scale(0.04 + peak * 6);
            //g.polygonLine();

            if(speakers[i].isPhantom){
                g.color(0.0,1.0,0.0);
            }else if(devChan == 0){
                g.color(1.0,0.0,0.0);
            }else if(devChan == 1){
                g.color(0.0,0.0,1.0);
            }else if(!speakers[i].enabled->get()){
                g.color(0.05,0.05,0.05);
            }else{
            g.color(1);
            }
            g.draw(mSpeakerMesh);
            g.popMatrix();
        }
        parameterGUI.draw(g);
    }

    bool onKeyDown(const Keyboard &k) override {
      if (k.alt()) {
        switch (k.key()) {
        case '1':
          presets.storePreset("preset1");
          std::cout << "Preset 1 stored." << std::endl;
          break;
        case '2':
          presets.storePreset("preset2");
          std::cout << "Preset 2 stored." << std::endl;
          break;
        case '3':
          presets.storePreset("preset3");
          std::cout << "Preset 3 stored." << std::endl;
          break;
        case '4':
          presets.storePreset("preset4");
          std::cout << "Preset 4 stored." << std::endl;
          break;
        }
      }
      else {

          int a = 1;
          string str =to_string(a);

        if(k.keyAsNumber() < 10 && k.keyAsNumber() >= 0){

            string presetString = "preset";
            presetString.append(to_string(k.keyAsNumber()));
            presets.recallPreset(presetString);
            std::cout << presetString + " loaded." << std::endl;
        }
        initPanner();
      }
    }
};

int main(){
    MyApp app;
    AudioDevice::printAll();

     //audioRate audioBlockSize audioOutputs audioInputs device

    //app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 60, 0, 1);
    
    //-1 for audioOutputs or audioInputs opens all available channels. see: AudioIO::channels()
//    app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 60, 0, 2);

    // Use this for 2809                   **CHANGE AUDIO INPUT TO 0 **
    //app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 2, 0, AudioDevice("Aggregate Device").id());

    AudioDevice dev = AudioDevice("Aggregate Device").id();
    app.configureAudio(dev, SAMPLE_RATE, BLOCK_SIZE, 2, 0);

    // Use this for sphere
    //    app.initAudio(44100, BLOCK_SIZE, -1, -1, AudioDevice("ECHO X5").id());

    //cout << "before AppStart" << endl;
    app.start();
    return 0;
}
