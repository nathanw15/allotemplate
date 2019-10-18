 #define GAMMA_H_INC_ALL         // define this to include all header files
#define GAMMA_H_NO_IO           // define this to avoid bringing AudioIO from Gamma

#include "Gamma/Gamma.h"
#include "al/core.hpp"
#include "al/core/sound/al_Speaker.hpp"
#include "al/util/ui/al_Parameter.hpp"
#include "al/util/ui/al_ParameterServer.hpp"
#include "Gamma/SamplePlayer.h"
#include "al/util/ui/al_ControlGUI.hpp"

#include "al/util/ui/al_HtmlInterfaceServer.hpp"

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

//ParameterServer paramServer("127.0.0.1",8080);
//Parameter srcAzimuth("srcAzimuth","",-0.5,"",-1.0*M_PI,M_PI);
ParameterBool updatePanner("updatePanner","",0.0);

float radius = 5.0;


//Parameter sourceSound("sourceSound","",0.0,"",0.0,3.0);
//Parameter soundFileIdx("soundFileIdx","",0.0,"",0.0,3.0);

ParameterBool sampleWise("sampleWise","",0.0);
ParameterBool useDelay("useDelay","", 0.0);

Parameter masterGain("masterGain","",0.5,"",0.0,1.0);
ParameterBool soundOn("soundOn","",0.0);

PresetHandler presets("../data/presets");
PresetHandler srcPresets("data/srcPresets");

SearchPaths searchpaths;

vector<string> files;//{"count.wav","lowBoys.wav","midiPiano.wav","pianoA.wav","pianoB.wav","pianoC.wav","pianoD.wav","pizzPhase.wav","hornPhase.wav"};

vector<string> posUpdateNames{"off", "trajectory", "moving"};

Parameter maxDelay("maxDelay","",0.0,"",0.0,1.0);

ParameterBool resetSamples("resetSamples","",0.0);

ParameterMenu setAllPosUpdate("setAllPosUpdate","",0,"");
ParameterMenu setAllSoundFileIdx("setAllSoundFileIdx","",0,"");
ParameterBool setAllEnabled("setAllEnabled","",0.0);

ParameterBool setPiano("setPiano","",0.0);

Parameter setAllAzimuth("setAllAzimuth","",2.9,"",-1.0*M_PI,M_PI);
Parameter azimuthSpread("azimuthSpread","",0.f,"",0.0,2.0*M_PI);

ParameterBool setAllRatesToOne("setAllRatesToOne","",0.0);

Parameter setPlayerPhase("setPlayerPhase","",0.0,"",0.0,44100.0);
ParameterBool triggerAllRamps("triggerAllRamps","",0.0);
ParameterBool setAllStartAzi("setAllStartAzi","",0.0);
ParameterBool setAllEndAzi("setAllEndAzi","",0.0);
Parameter setAllDurations("setAllDurations","",0.0,"",0.0,10.f);


ParameterBool combineAllChannels("combineAllChannels","",0.0);
//HtmlInterfaceServer interfaceServer("/Users/primary1/Documents/code/allolibCode/projects/interface.js");

mutex enabledSpeakersLock;

struct Ramp {

    unsigned int startSample, endSample;
    bool done = false;
    bool running = false;
    bool trigger = false;

    ParameterBundle rampBundle{"rampBundle"};
    ParameterBool triggerRamp{"triggerRamp","",0.0};
    Parameter rampStartAzimuth{"rampStartAzimuth","",-0.5,"",-1.0*M_PI,M_PI};
    Parameter rampEndAzimuth{"rampEndAzimuth","",0.5,"",-1.0*M_PI,M_PI};
    Parameter rampDuration{"rampDuration", "",1.0,"",0.0,10.0};

    Ramp(){


        rampBundle  << triggerRamp << rampStartAzimuth << rampEndAzimuth << rampDuration;

//        triggerRamp.registerChangeCallback([&](float val){
//            if(val == 1.f){ // is this correct way to check?
//                //cout << "Ramp triggered" << endl;
//                trigger = true;
//            }
//        });
    }

    void set(float startAzi, float endAzi, float dur){
        rampStartAzimuth.set(startAzi);
        rampEndAzimuth.set(endAzi);
        rampDuration.set(dur);
    }

    void start(unsigned int startSamp){
        startSample = startSamp;
        endSample = startSamp +  rampDuration.get() *SAMPLE_RATE;
        running = true;
        done = false;
        //cout << startSamp << " endSamp: " << endSample << endl;
    }

    float next(unsigned int sampleNum){

        if(trigger){
            trigger = false;
            start(sampleNum);
        }

        if(!done && !running){
            return rampStartAzimuth.get();
        }else if(!done && running){

            if(sampleNum > endSample){
                sampleNum = endSample;
                done = true;
                running = false;
            }
            float val = (((sampleNum - startSample) * (rampEndAzimuth.get() - rampStartAzimuth.get())) / (endSample - startSample)) + rampStartAzimuth.get();
            return val;
        } else {
            return rampEndAzimuth.get();
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
    //int sound = 0;
    //Parameter fileIdx{"fileIdx","",2.0,"",0.0,3.0};
    ParameterBundle vsBundle{"vsBundle"};
    float buffer[BLOCK_SIZE];
    Parameter angularFreq {"angularFreq","",1.f,"",-15.f,15.f};
    Parameter samplePlayerRate {"samplePlayerRate","",1.f,"",1.f,1.5f};

    ParameterMenu fileMenu{"fileMenu","",0,""};

    Ramp sourceRamp;

    int previousSamp = 0;

    VirtualSource(){

//        vector<string> posUpdateNames = {"off", "trajectory", "moving"};
        positionUpdate.setElements(posUpdateNames);

        fileMenu.setElements(files);

//        samplePlayer.load(searchpaths.find(files[fileMenu.get()]).filepath().c_str());

        samplePlayer.load("src/sounds/count.wav");


//        samplePlayer.min(1.1);
        enabled.setHint("latch", 1.f);
       // fileIdx.setHint("intcombo",1.f);
        //positionUpdate.setHint("intcombo",1.f);

        //cout << vsBundle.bundleIndex() << endl;
        samplePlayerRate.set(1.0 + (.002 * vsBundle.bundleIndex()));
        samplePlayer.rate(samplePlayerRate.get());

        aziInRad.setProcessingCallback([&](float val){
            wrapValues(val);
            return val;
        });

        samplePlayerRate.registerChangeCallback([&](float val){
           samplePlayer.rate(val);
        });

        fileMenu.registerChangeCallback([&](float val){
            cout << "FileMenu val: " << val << endl;
            samplePlayer.load(searchpaths.find(files[val]).filepath().c_str());
        });

//        fileIdx.registerChangeCallback([&](float val){
//            samplePlayer.load(searchpaths.find(files[val]).filepath().c_str());
//        });

        sourceRamp.triggerRamp.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                //cout << "Ramp triggered" << endl;
                sourceRamp.trigger = true;
                samplePlayer.reset();
            }
        });
        vsBundle << enabled << sourceGain << aziInRad << positionUpdate << fileMenu << samplePlayerRate << sourceRamp.triggerRamp << sourceRamp.rampStartAzimuth << sourceRamp.rampEndAzimuth << sourceRamp.rampDuration << angularFreq;
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

    void getBuffer(unsigned int sampleNumber, float *buffer){
        updatePosition(sampleNumber);
        for(int i = 0; i < BLOCK_SIZE; i++){
            if(samplePlayer.done()){
                samplePlayer.reset();
            }
            buffer[i] = sourceGain.get() * samplePlayer();
        }
    }

    float getSample(unsigned int sampleNumber){
        updatePosition(sampleNumber);
        if(samplePlayer.done()){
            samplePlayer.reset();
        }
        return sourceGain.get() * samplePlayer();
    }

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
    float aziInRad;

    int delay;
    void *buffer;
    int bufferSize;
    int readPos,writePos;

    bool isPhantom = false;

    SpeakerV(int chan, float az=0.f, float el=0.f, int gr=0, float rad=1.f, float ga=1.f, int del = 0){
        delay = del;
        readPos = 0;
//        writePos = delay;
        writePos = 0;

        setDelay(maxDelay.get()*SAMPLE_RATE);
        bufferSize = 44100*2;//MAKE WORK WITH MAX DELAY

        buffer = calloc(bufferSize,sizeof(float));
        //cout << "Speaker: " << chan << " Write Pos: " << writePos << endl;
         //buffer = new float[bufferSize]();

      // buffer = new gam::Array<float>();
      // buffer.resize(delay*2);

        deviceChannel = chan;
        azimuth= az;
        elevation = el;
        group = gr;
        radius = rad;
        gain = ga;
        aziInRad = toRad(az);

        oscTag = "speaker"+ std::to_string(deviceChannel) + "/enabled";
        enabled = new ParameterBool(oscTag,"",1.0);
        enabled->setHint("latch",1.f);

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
//            writePos = 0;
            writePos -= bufferSize;
        }

        int writeDelay = writePos + delay;
        if(writeDelay >= bufferSize){
            writeDelay -= bufferSize;
        }

        float *b = (float*)buffer;
//        b[writePos] = samp;
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

//void presetCB(int val, void *s, void *c){
//    cout << "registerPresetCallback" << endl;
//    initPanner();
//}

class MyApp : public App
{
public:
    Mesh mSpeakerMesh;
    vector<Mesh> mVec;
    vector<int>  sChannels;
    SpeakerLayout speakerLayout;
    atomic<float> *mPeaks {nullptr};
    float speedMult = 0.03f;
    Vec3d srcpos {0.0,0.0,0.0};
    //Ramp linearRamp;
   // vector<gam::SamplePlayer<>*> samplePlayers;
//    SearchPaths searchpaths;
    ControlGUI parameterGUI;

    ParameterBundle xsetAllBundle{"xsetAllBundle"};

    MyApp()
    {

        cout << "Constructor" << endl;
        searchpaths.addAppPaths();
//        searchpaths.addRelativePath("../sounds");
        searchpaths.addRelativePath("src/sounds");
        searchpaths.print();
        //itemListInDir("../sounds").print();
//        FileList fl = itemListInDir("../sounds");
        FileList fl = itemListInDir("src/sounds");
        for(int i = 0; i < fl.count(); i++){
            string fPath = fl[i].filepath();
            fPath = fPath.substr(fPath.find_last_of("/\\")+1);
            cout << fPath << endl;
            files.push_back(fPath);
        }

        cout << "Path of Count " << searchpaths.find("count.wav").filepath().c_str() << endl;
        parameterGUI << soundOn << resetSamples << updatePanner << sampleWise << useDelay << masterGain << maxDelay << combineAllChannels;
        parameterGUI << srcPresets;

        xsetAllBundle << setAllEnabled << setAllPosUpdate << setAllSoundFileIdx <<setAllAzimuth << azimuthSpread << setAllRatesToOne << setPlayerPhase << triggerAllRamps << setAllStartAzi << setAllEndAzi << setAllDurations << setPiano;
        parameterGUI << xsetAllBundle;



//        parameterServer() << soundOn << resetSamples << updatePanner << sampleWise << useDelay << masterGain << maxDelay;
//        interfaceServer << parameterServer();


        for(int i = 0; i < NUM_SOURCES; i++){
            auto *newVS = new VirtualSource; // This memory is not freed and it should be...
            sources.push_back(newVS);
            // Register its parameter bundle with the ControlGUI
            parameterGUI << newVS->vsBundle;
            //paramServer << newVS->vsBundle;
        }
//        xsetAllBundle << setAllEnabled << setAllPosUpdate << setAllSoundFileIdx <<setAllAzimuth << azimuthSpread << setAllRatesToOne << setPlayerPhase << triggerAllRamps << setAllStartAzi << setAllEndAzi << setAllDurations;
//        parameterGUI << xsetAllBundle;

        //parameterServer() << soundOn << resetSamples << updatePanner << sampleWise << useDelay << masterGain << maxDelay << setAllEnabled << setAllPosUpdate << setAllSoundFileIdx <<setAllAzimuth << azimuthSpread << setAllRatesToOne << setPlayerPhase;

       //interfaceServer << parameterServer();

       // presets.registerPresetCallback(presetCB);

       // paramServer << srcAzimuth << updatePanner << triggerRamp << rampStartAzimuth << rampEndAzimuth << rampDuration << sourceSound << soundFileIdx << sampleWise << useDelay << masterGain << useRamp << soundOn;

        //paramServer.print();
        soundOn.setHint("latch", 1.f);
        sampleWise.setHint("latch", 1.f);
        useDelay.setHint("latch",1.f);
        //sourceSound.setHint("intcombo",1.f);
        //soundFileIdx.setHint("intcombo",1.f);


        parameterServer() << soundOn << resetSamples << updatePanner << sampleWise << useDelay << masterGain << maxDelay;
        //interfaceServer << parameterServer();

        setAllEnabled.setHint("latch",1.f);
        setPiano.setHint("latch",1.f);
        //setAllPosUpdate.setHint("intcombo",1.f);
        //setAllSoundFileIdx.setHint("intcombo",1.f);
        setAllRatesToOne.setHint("latch",1.f);
        setAllPosUpdate.setElements(posUpdateNames);

        setAllSoundFileIdx.setElements(files);

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
        
        setPiano.registerChangeCallback([&](float val){
            for(VirtualSource *v: sources){
               // v->vsBundle.bundleIndex()
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
//                if(v->vsBundle.bundleIndex() == 0){
//                    v->samplePlayer.load("sounds/pianoA.wav");
//
//                }else
                //v->enabled.set(val);
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
                    v->sourceRamp.triggerRamp.set(1.f);
                }
            }
        });

        setAllStartAzi.registerChangeCallback([&](float val){
            if(val == 1.f){
                for(VirtualSource *v: sources){
                    v->sourceRamp.rampStartAzimuth.set(v->aziInRad);
                }
            }
        });

        setAllEndAzi.registerChangeCallback([&](float val){
            if(val == 1.f){
                for(VirtualSource *v: sources){
                    v->sourceRamp.rampEndAzimuth.set(v->aziInRad);
                }
            }
        });

        setAllDurations.registerChangeCallback([&](float val){

            for(VirtualSource *v: sources){
                v->sourceRamp.rampDuration.set(val);
            }

        });

        updatePanner.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                cout << "panner Updated" << endl;
                initPanner();
            }
        });

        maxDelay.registerChangeCallback([&](float val){
            //cout << maxDelay.get() << " " << val << endl;
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

//        srcpos = ambiSphericalToOGLCart(srcAzi,radius);
//        Vec3f ambiCartSrcPos = srcpos;

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
            //speakerChan1 = speakerChan2 = -1;
            //cout << "enabled Speaker Size " + enabledSpeakers.size() << endl;
        }else{
            speakerChan1 = speakerChan2 = -1;
//            //check if source is beyond the first or last speaker
//            if(srcAzi < enabledSpeakers[0]->aziInRad){
//                speakerChan1 = enabledSpeakers[0]->deviceChannel;
//                speakerChan2 = enabledSpeakers[0+1]->deviceChannel;
//                speakSrcAngle = fabsf(enabledSpeakers[0]->aziInRad - srcAzi);
//                gains.x = 1.f / (radius * (M_PI - speakSrcAngle));

//            } else if(srcAzi > enabledSpeakers[enabledSpeakers.size()-1]->aziInRad){
//                speakerChan1 = enabledSpeakers[enabledSpeakers.size()-2]->deviceChannel;//set to speaker before last
//                speakerChan2 = enabledSpeakers[enabledSpeakers.size()-1]->deviceChannel;
//                speakSrcAngle = fabsf(enabledSpeakers[enabledSpeakers.size()-1]->aziInRad - srcAzi);
//                linearDistance = 2.0*radius*cos((M_PI - speakSrcAngle)/2.0);
//                gains.y = 1.f / (radius * (M_PI - speakSrcAngle));

//            } else{//Source between first and last speakers
//                for(int i = 0; i < enabledSpeakers.size()-1; i++){
//                    speakerChan1 = enabledSpeakers[i]->deviceChannel;
//                    speakerChan2 = enabledSpeakers[i+1]->deviceChannel;
//                    if(srcAzi == enabledSpeakers[i]->aziInRad ){
//                        gains.x = 1.0;
//                        break;
//                    }else if(srcAzi > enabledSpeakers[i]->aziInRad && srcAzi < enabledSpeakers[i+1]->aziInRad){
//                        createMatrix(enabledSpeakers[i]->vec(),enabledSpeakers[i+1]->vec());
//                        invert(matrix);
//                        for (unsigned i = 0; i < 2; i++){
//                            for (unsigned j = 0; j < 2; j++){
//                                gains[i] += ambiCartSrcPos[j] * matrix(j,i);
//                            }
//                        }
//                        gains = gains.normalize();
//                        break;
//                    } else if(srcAzi == enabledSpeakers[i+1]->aziInRad){
//                        gains.y = 1.0;
//                        break;
//                    }
//                }
//            }
        }
        return gains;
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

    void renderSample(AudioIOData &io, const float &srcAzi, const float &sample){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io,srcAzi, speakerChan1, speakerChan2);
        setOutput(io,speakerChan1,io.frame(),sample * gains[0]);
        setOutput(io,speakerChan2,io.frame(),sample * gains[1]);
    }

    void onInit() override {

        cout << "onInit()" << endl;

//        searchpaths.addAppPaths();
//        searchpaths.addRelativePath("../sounds");

//        samplePlayers.push_back(new gam::SamplePlayer<>(searchpaths.find("lowBoys.wav").filepath().c_str()));
//        samplePlayers.push_back(new gam::SamplePlayer<>(searchpaths.find("count.wav").filepath().c_str()));
//        samplePlayers.push_back(new gam::SamplePlayer<>(searchpaths.find("devSamples.wav").filepath().c_str()));

        //linearRamp.set(-2.0,2.0,linearRamp.rampDuration.get());


       // SpeakerV(0,90.0,0.0,0,5.0,1.0,0);
       // speakers.push_back(SpeakerV(0,90.0 - (10.0*0),0.0,0,5.0,1.0,0));

        float ang;
        for (int i = 0; i < 32; i++){
            int delay = rand() % static_cast<int>(MAX_DELAY + 1);
            ang = 170.0 - (11.0*i);
            speakers.push_back(SpeakerV(i,ang,0.0,0,5.0,0,delay));
        }

        //-1 for phantom channels (can remove isPhantom and just check -1)
        SpeakerV s(-1, 175,0.0,0,5.0,0,0);
        s.isPhantom = true;
        speakers.push_back(s);

        SpeakerV p(-1, ang-10.0,0.0,0,5.0,0,0);
        p.isPhantom = true;
        speakers.push_back(p);
       // speakers.push_back(SpeakerV(-1, -100,0.0,0,5.0,0,0));

        initPanner();

        for(int i = 0; i < speakers.size(); i++){
            parameterGUI << speakers[i].enabled;


            presets << *speakers[i].enabled;
            //speakers[i].enabled->setHint("latch",1.f);
        }

        int highestChannel = 0;
        for(SpeakerV s:speakers){
            if((int) s.deviceChannel > highestChannel){
                highestChannel = s.deviceChannel;
            }
        }

        audioIO().close();
        audioIO().channelsOut(highestChannel + 1);
        audioIO().open();

        mPeaks = new atomic<float>[highestChannel + 1];

        addSphere(mSpeakerMesh, 1.0, 5, 5);

    }

    void onCreate() override {
        cout << "onCreate()" << endl;
        nav().pos(0, 1, 20);
        parameterGUI.init();
        //initIMGUI();
    }

//    void onExit() override {
//        shutdownIMGUI();
//    }

    void onAnimate( double dt) override {
        navControl().active(!parameterGUI.usingInput());
    }

    virtual void onSound(AudioIOData &io) override {

        static unsigned int t = 0;
        //double sec;
        float srcBuffer[BLOCK_SIZE];

        float enabledSources = 0.0f;

        float mGain = masterGain.get();
        gam::Sync::master().spu(audioIO().fps());

        if(soundOn.get() > 0.5){
            while (io()) {
                //int i = io.frame();
//                if(sourceSound.get() == 0){
//                    float env = (22050 - (t % 22050))/22050.0;
//                    srcBuffer[i] = mGain * rnd::uniform() * env;
//                } else if(sourceSound.get() ==1){
//                    gam::SamplePlayer<> *player = samplePlayers[soundFileIdx];
//                    if(player->done()){
//                        player->reset();
//                    }
//                    srcBuffer[i] = mGain * player->operator ()();
//                }
                ++t;

                if(sampleWise.get() == 1.f){

                    for(VirtualSource *v: sources){
                        if(v->enabled){
                            enabledSources += 1.0f;
                            float sample = v->getSample(t);
                            if(useDelay.get() == 1.f){
                                renderSampleDelaySpeakers(io,v->aziInRad.get(),sample);
                            } else {
                                renderSample(io,v->aziInRad.get(), sample);
                            }
                        }
                    }
                }
            }

            if(sampleWise.get() == 0.f){

                for(VirtualSource *v: sources){
                    if(v->enabled){
                        enabledSources += 1.0f;
                        v->getBuffer(t,srcBuffer);
                        if(useDelay.get() == 1.f){
                            renderBufferDelaySpeakers(io,v->aziInRad.get(), srcBuffer);
                        }else{
                            renderBuffer(io,v->aziInRad.get(), srcBuffer);
                        }
                    }
                }
            }



            if(combineAllChannels.get() == 1.0f){
                float combineBuffer[BLOCK_SIZE];
                for(int i = 0; i < BLOCK_SIZE;i++){
                    combineBuffer[i] = 0.0f;

                }
                        //combine all the channels into one buffer
                        for (int speaker = 0; speaker < speakers.size(); speaker++) {
                            if(!speakers[speaker].isPhantom){
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
                            if(!speakers[speaker].isPhantom){
                                int deviceChannel = speakers[speaker].deviceChannel;

                                for (int i = 0; i < io.framesPerBuffer(); i++) {
                                     if(deviceChannel < io.channelsOut()) {
                                         io.out(deviceChannel,i) = combineBuffer[i]*masterGain.get();
                                        
                                    }
                                }

                            }
                        }

               

            }
//            if(sampleWise.get() == 0.f){
//                if(useDelay.get() == 1.f){
//                    renderBufferDelaySpeakers(io,srcAzimuth.get(), srcBuffer);
//                }else{
//                    renderBuffer(io,srcAzimuth.get(), srcBuffer);
//                }
//            }
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
            g.translate(v->getSamplePlayerPhase(),v->vsBundle.bundleIndex()*0.25,0.0);
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
            float peak = 0.0;
            if(!speakers[i].isPhantom){
                peak = mPeaks[devChan].load();
            }
            g.scale(0.04 + peak * 6);
            g.polygonLine();

            if(speakers[i].isPhantom){
                g.color(0.0,1.0,0.0);
            }else if(devChan == 0){
                g.color(1.0,0.0,0.0);
            }else if(!speakers[i].enabled->get()){
                g.color(0.05,0.05,0.05);
            }else{
            g.color(1);
            }
            g.draw(mSpeakerMesh);
            g.popMatrix();
        }
        parameterGUI.draw(g);

//        beginIMGUI();
//        ParameterGUI::beginPanel("Speakers");

//        for(int i = 0; i < speakers.size(); i++){
//            ParameterBool* spk = speakers[i].enabled;
//            ParameterGUI::drawParameterMeta(spk);
//        }
//        //ParameterGUI::drawParameterMeta(&y);
//        //ParameterGUI::drawParameterMeta(&z);
//        //ParameterGUI::endPanel();

//        endIMGUI();
    }

    virtual void onKeyDown(const Keyboard &k) override {
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
          cout << k.key() << endl;
//        switch (k.key()) {
        if(k.keyAsNumber() < 10 && k.keyAsNumber() >= 0){

            string presetString = "preset";
            //presetString.
           // presetString = strcat(presetString,to_string(k.key()));
           // strc
            presetString.append(to_string(k.keyAsNumber()));
            //presetString.push_back(to_string(k.key()));
            presets.recallPreset(presetString);
            std::cout << presetString + " loaded." << std::endl;
        }
//        case '1':
//          presets.recallPreset("preset1");
//          std::cout << "Preset 1 loaded." << std::endl;
//          break;
//        case '2':
//          presets.recallPreset("preset2");
//          std::cout << "Preset 2 loaded." << std::endl;
//          break;
//        case '3':
//          presets.recallPreset("preset3");
//          std::cout << "Preset 3 loaded." << std::endl;
//          break;
//        case '4':
//          presets.recallPreset("preset4");
//          std::cout << "Preset 4 loaded." << std::endl;
//          break;
        //}
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
    app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 2, 0, AudioDevice("Aggregate Device").id());

    // Use this for sphere
    //    app.initAudio(44100, BLOCK_SIZE, -1, -1, AudioDevice("ECHO X5").id());

    cout << "before AppStart" << endl;
    app.start();
    return 0;
}
