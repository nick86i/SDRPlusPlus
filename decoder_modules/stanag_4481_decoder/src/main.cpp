#include <core.h>
#include <gui/gui.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

SDRPP_MOD_INFO{"stanag_4481_decoder", "75-baud wide-shift FSK analyzer", "Nick / OpenAI", 0, 2, 0, -1};

namespace { constexpr double RATE=8000.0, BANDWIDTH=3000.0, BAUD=75.0, TONE=425.0, PI=3.14159265358979323846; }

class Stanag4481Module : public ModuleManager::Instance {
public:
    explicit Stanag4481Module(std::string instanceName) : name(std::move(instanceName)) {
        createVFO(); sink.init(vfo->output, handler, this); sink.start();
        gui::menu.registerEntry(name, menuHandler, this, this);
    }
    ~Stanag4481Module() override { stopCapture(); gui::menu.removeEntry(name); if(enabled){sink.stop();sigpath::vfoManager.deleteVFO(vfo);} }
    void postInit() override {}
    void enable() override { if(enabled)return;createVFO();sink.setInput(vfo->output);sink.start();enabled=true; }
    void disable() override { if(!enabled)return;sink.stop();sigpath::vfoManager.deleteVFO(vfo);vfo=nullptr;enabled=false; }
    bool isEnabled() override { return enabled; }

private:
    void createVFO(){vfo=sigpath::vfoManager.createVFO(name,ImGui::WaterfallVFO::REF_CENTER,0,BANDWIDTH,RATE,BANDWIDTH,BANDWIDTH,true);vfo->setSnapInterval(100);}
    static void menuHandler(void* ctx){
        auto* s=static_cast<Stanag4481Module*>(ctx); if(!s->enabled)return;
        ImGui::TextUnformatted("Wide-Shift FSK Analyzer"); ImGui::Separator();
        ImGui::Text("Observed preset: 75 baud / 850 Hz shift");
        ImGui::TextWrapped("Protocol: unidentified; framing does not match documented STANAG 4481/KG-84 structure.");
        const float q=s->quality.load();
        if(s->protocolLocked.load()) ImGui::TextColored(ImVec4(.2f,1,.2f,1),"Decode gate: OPEN (sync verified)"); else ImGui::TextUnformatted("Decode gate: closed (noise ignored)");
        ImGui::Text("Tone concentration: %.1f%%",s->toneConcentration.load()*100.0f);
        if(q>0.35f&&s->carrierPresent.load()) ImGui::TextColored(ImVec4(.2f,1,.2f,1),"Signal lock: LOCKED"); else ImGui::TextUnformatted("Signal lock: searching");
        ImGui::Text("Settled tone confidence: %.0f%%",q*100.0f);
        ImGui::Text("Bits: %llu   Ones: %.1f%%",(unsigned long long)s->bitCount.load(),s->oneRatio.load()*100.0f);
        bool reverse=s->reverse.load(); if(ImGui::Checkbox("Reverse polarity",&reverse))s->reverse.store(reverse);
        if(ImGui::Button("Clear analysis")){std::lock_guard<std::mutex> l(s->dataMutex);s->bytes.clear();s->recentBits.clear();s->recentReliability.clear();s->cycleHistory.clear();s->lastFrameHex.clear();s->bitCount=0;s->ones=0;s->transitionCount=0;s->longestRun=0;s->acceptedFrames=0;s->rejectedSync=0;s->syncErrorTotal=0;s->lastFrameBit=0;s->validFrames=0;s->failedFrames=0;s->uncertainFrames=0;s->correctedFrames=0;s->uncorrectableFrames=0;s->cycleComparisons=0;s->cycleExact=0;s->cycleNear=0;s->cycleDistanceTotal=0;}
        ImGui::SameLine();
        if(!s->capturing.load()) { if(ImGui::Button("Start bitstream capture"))s->startCapture(); }
        else if(ImGui::Button("Stop capture"))s->stopCapture();
        if(s->capturing.load()) ImGui::TextColored(ImVec4(1,.35f,.25f,1),"Recording: %llu bytes",(unsigned long long)s->capturedBytes.load());
        else if(!s->capturePath.empty()) ImGui::TextWrapped("Last capture: %s",s->capturePath.c_str());
        const uint64_t age=s->bitCount.load()-s->lastFrameBit.load();
        if(s->acceptedFrames.load()&&age<300)ImGui::TextColored(ImVec4(.2f,1,.2f,1),"113-bit frame lock: LOCKED");else ImGui::TextUnformatted("113-bit frame lock: searching");
        ImGui::Text("Frames: %llu accepted / %llu rejected",(unsigned long long)s->acceptedFrames.load(),(unsigned long long)s->rejectedSync.load());
        if(s->acceptedFrames.load())ImGui::Text("Average sync errors: %.2f",double(s->syncErrorTotal.load())/s->acceptedFrames.load());
        ImGui::Text("Inferred checks: %llu valid / %llu failed / %llu uncertain",(unsigned long long)s->validFrames.load(),(unsigned long long)s->failedFrames.load(),(unsigned long long)s->uncertainFrames.load());
        ImGui::Text("Cautious correction: %llu corrected / %llu uncorrectable",(unsigned long long)s->correctedFrames.load(),(unsigned long long)s->uncorrectableFrames.load());
        const uint64_t cycleCount=s->cycleComparisons.load();
        if(cycleCount)ImGui::TextWrapped("Provisional superframe: 85 x 113 = 9605 bits (128.07 s); cycle match %.1f%% near / %.1f%% exact, average payload distance %.2f bits",
            100.0*s->cycleNear.load()/cycleCount,100.0*s->cycleExact.load()/cycleCount,double(s->cycleDistanceTotal.load())/cycleCount);
        else ImGui::TextUnformatted("Provisional superframe: waiting for 86 frames");
        std::string frameText;{std::lock_guard<std::mutex> l(s->dataMutex);frameText=s->lastFrameHex;}
        if(!frameText.empty())ImGui::TextWrapped("Latest frame: %s",frameText.c_str());
        s->drawAnalysis();
        ImGui::Separator(); ImGui::TextUnformatted("Recent demodulated bytes:");
        std::string text; {std::lock_guard<std::mutex> l(s->dataMutex);std::ostringstream o;o<<std::hex<<std::uppercase<<std::setfill('0');for(auto b:s->bytes)o<<std::setw(2)<<(int)b<<' ';text=o.str();}
        ImGui::TextWrapped("%s",text.empty()?"Waiting for FSK data...":text.c_str());
        ImGui::Spacing(); ImGui::TextWrapped("Center the VFO midway between the two carriers. Raw bytes may be unreadable when the transmission carries KG-84 encrypted traffic.");
    }
    void drawAnalysis(){
        std::vector<uint8_t> b; std::vector<uint8_t> bits,reliability;
        {std::lock_guard<std::mutex> l(dataMutex);b=bytes;bits=recentBits;reliability=recentReliability;}
        if(b.empty())return;
        int hist[256]={}; for(uint8_t v:b)hist[v]++;
        int dominant=0; double entropy=0; for(int i=0;i<256;i++){if(hist[i]>hist[dominant])dominant=i;if(hist[i]){double p=double(hist[i])/b.size();entropy-=p*std::log2(p);}}
        std::map<uint16_t,int> words; uint16_t reg=0; for(size_t i=0;i<bits.size();i++){reg=uint16_t((reg<<1)|bits[i]);if(i>=15)words[reg]++;}
        uint16_t sync=0;int syncCount=0;for(const auto& p:words)if(p.second>syncCount){sync=p.first;syncCount=p.second;}
        const uint64_t count=bitCount.load(), transitions=transitionCount.load();
        ImGui::Separator(); ImGui::TextUnformatted("Protocol analysis");
        ImGui::Text("Entropy: %.2f / 8 bits",entropy);
        ImGui::Text("Transitions: %.1f%%   Longest run: %llu bits",count>1?100.0*transitions/(count-1):0.0,(unsigned long long)longestRun.load());
        ImGui::Text("Dominant byte: %02X (%.1f%%)",dominant,100.0*hist[dominant]/b.size());
        if(bits.size()>=64)ImGui::Text("Recurring 16-bit candidate: %04X (%d hits)",sync,syncCount);
        const double dominantRatio=double(hist[dominant])/b.size();
        if(dominantRatio>.45||longestRun.load()>40)ImGui::TextColored(ImVec4(.9f,.8f,.2f,1),"Assessment: idle or repetitive framing");
        else if(b.size()>=64&&entropy>7.0)ImGui::TextColored(ImVec4(.9f,.6f,.2f,1),"Assessment: high entropy; encrypted/scrambled likely");
        else ImGui::TextUnformatted("Assessment: structured data; more capture needed");
        if(!reliability.empty()){double avg=0;size_t weak=0;for(uint8_t r:reliability){avg+=r;if(r<128)weak++;}ImGui::Text("Soft-bit confidence: %.0f%% avg; %.1f%% weak",avg/reliability.size()/2.55,100.0*weak/reliability.size());}
        drawFramingAnalysis(bits,reliability);
    }
    void drawFramingAnalysis(const std::vector<uint8_t>& bits,const std::vector<uint8_t>& reliability){
        // Treat a run of at least 20 space bits (267 ms at 75 baud) as an
        // inter-burst steady-tone interval. Ignore incomplete edge regions.
        std::vector<std::pair<size_t,size_t>> regions;std::vector<size_t> starts;
        size_t regionStart=0,i=0;bool haveLeadingGap=false;
        while(i<bits.size()){
            if(bits[i]){i++;continue;}
            size_t j=i;while(j<bits.size()&&!bits[j])j++;
            if(j-i>=20){if(haveLeadingGap&&i>regionStart&&i-regionStart>=32){regions.emplace_back(regionStart,i);starts.push_back(regionStart);}regionStart=j;haveLeadingGap=true;}
            i=j;
        }
        if(regions.size()<2)return;
        std::vector<double> lengths,intervals;for(auto r:regions)lengths.push_back(double(r.second-r.first));
        for(size_t n=1;n<starts.size();n++)intervals.push_back(double(starts[n]-starts[n-1]));
        auto median=[](std::vector<double> v){std::sort(v.begin(),v.end());return v.empty()?0.0:v[v.size()/2];};
        const double medLength=median(lengths),medInterval=median(intervals);

        struct Candidate{int lag;double match;};std::vector<Candidate> candidates;
        for(int lag=24;lag<=196;lag++){
            double same=0,total=0;
            for(auto r:regions){if(r.second-r.first<=size_t(lag))continue;for(size_t k=r.first;k+lag<r.second;k++){const uint8_t lower=reliability[k]<reliability[k+lag]?reliability[k]:reliability[k+lag];const double w=lower/255.0;if(w<.25)continue;same+=(bits[k]==bits[k+lag])*w;total+=w;}}
            if(total>=100)candidates.push_back({lag,same/total});
        }
        std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.match>b.match;});
        std::vector<Candidate> selected;for(auto c:candidates){bool adjacent=false;for(auto s:selected)if(std::abs(c.lag-s.lag)<=2)adjacent=true;if(!adjacent)selected.push_back(c);if(selected.size()==3)break;}
        ImGui::Separator();ImGui::TextUnformatted("Burst / framing analysis");
        ImGui::Text("Complete bursts in window: %d",(int)regions.size());
        ImGui::Text("Median burst: %.0f bits (%.2f s)",medLength,medLength/BAUD);
        if(medInterval>0)ImGui::Text("Median repetition: %.0f bits (%.2f s)",medInterval,medInterval/BAUD);
        if(!selected.empty()){
            std::ostringstream o;o<<"Frame candidates: ";for(size_t n=0;n<selected.size();n++){if(n)o<<", ";o<<selected[n].lag<<" bit ("<<std::fixed<<std::setprecision(0)<<selected[n].match*100<<"% match)";}
            ImGui::TextWrapped("%s",o.str().c_str());
            ImGui::TextWrapped("Candidates are correlation measurements, not confirmed protocol frame sizes.");
            drawSyncAnalysis(bits,reliability,regions,selected.front().lag);
        }
    }
    struct BitOccurrence{size_t region;size_t position;};
    static int hamming32(uint32_t v){int n=0;while(v){v&=v-1;n++;}return n;}
    void drawSyncAnalysis(const std::vector<uint8_t>& bits,const std::vector<uint8_t>& reliability,const std::vector<std::pair<size_t,size_t>>& regions,int fallbackBits){
        std::map<uint32_t,std::vector<BitOccurrence>> occurrences;
        for(size_t ri=0;ri<regions.size();ri++){
            auto r=regions[ri];if(r.second-r.first<32)continue;uint32_t word=0;
            for(size_t k=r.first;k<r.second;k++){word=(word<<1)|bits[k];if(k-r.first>=31&&word!=0&&word!=0xFFFFFFFFu)occurrences[word].push_back({ri,k-r.first-31});}
        }
        uint32_t bestWord=0;size_t bestRegions=0,bestCount=0;std::vector<BitOccurrence> bestOccurrences;
        for(const auto& item:occurrences){std::set<size_t> distinct;for(auto o:item.second)distinct.insert(o.region);
            if(distinct.size()>bestRegions||(distinct.size()==bestRegions&&item.second.size()>bestCount)){bestWord=item.first;bestRegions=distinct.size();bestCount=item.second.size();bestOccurrences=item.second;}}
        // Expand the exact candidate using reliable matches with up to three bit
        // errors. This retains sync through selective fading without accepting
        // arbitrary low-confidence noise as evidence.
        std::vector<BitOccurrence> fuzzy;uint64_t errorSum=0;double reliabilitySum=0;
        for(size_t ri=0;ri<regions.size();ri++){auto r=regions[ri];uint32_t word=0;int lastAccepted=-1000;
            for(size_t k=r.first;k<r.second;k++){word=(word<<1)|bits[k];if(k-r.first<31)continue;const int pos=(int)(k-r.first-31);if(pos-lastAccepted<8)continue;
                double rel=0;for(size_t n=k-31;n<=k;n++)rel+=reliability[n];rel/=32.0*255.0;const int errors=hamming32(word^bestWord);
                if(rel>=.50&&errors<=3){fuzzy.push_back({ri,(size_t)pos});lastAccepted=pos;errorSum+=errors;reliabilitySum+=rel;}}
        }
        if(fuzzy.size()>=bestOccurrences.size()){bestOccurrences=fuzzy;std::set<size_t> distinct;for(auto o:fuzzy)distinct.insert(o.region);bestRegions=distinct.size();bestCount=fuzzy.size();}
        int frameBits=fallbackBits;std::vector<int> spacing;
        for(size_t a=0;a<bestOccurrences.size();a++)for(size_t b=a+1;b<bestOccurrences.size();b++)if(bestOccurrences[a].region==bestOccurrences[b].region){
            int d=(int)bestOccurrences[b].position-(int)bestOccurrences[a].position;if(d>=24&&d<=196)spacing.push_back(d);}
        if(!spacing.empty()){std::sort(spacing.begin(),spacing.end());frameBits=spacing[spacing.size()/2];}
        if(bestRegions>=3){
            ImGui::Text("Sync candidate: %08X (%u bursts, %u hits)",bestWord,(unsigned)bestRegions,(unsigned)bestCount);
            if(!fuzzy.empty())ImGui::Text("Fuzzy sync: %.2f avg errors; %.0f%% confidence",double(errorSum)/fuzzy.size(),100.0*reliabilitySum/fuzzy.size());
            ImGui::Text("Anchor-derived frame spacing: %d bits",frameBits);
            drawAnchorStability(bits,reliability,regions,bestOccurrences,frameBits);
        }else drawStabilityMap(bits,regions,fallbackBits);
    }
    void drawAnchorStability(const std::vector<uint8_t>& bits,const std::vector<uint8_t>& reliability,const std::vector<std::pair<size_t,size_t>>& regions,const std::vector<BitOccurrence>& occurrences,int frameBits){
        if(frameBits<=0)return;std::vector<double> oneWeight(frameBits,0),totalWeight(frameBits,0);uint32_t blocks=0;
        for(auto o:occurrences){const auto r=regions[o.region];const size_t start=r.first+o.position;if(start+size_t(frameBits)>r.second)continue;blocks++;
            for(int pos=0;pos<frameBits;pos++){const double w=reliability[start+size_t(pos)]/255.0;oneWeight[pos]+=bits[start+size_t(pos)]*w;totalWeight[pos]+=w;}}
        if(blocks<2)return;std::string pattern;int stable=0,strong=0;
        for(int pos=0;pos<frameBits;pos++){const double p=totalWeight[pos]>0?oneWeight[pos]/totalWeight[pos]:.5;
            if(p>=.85){pattern.push_back('1');stable++;if(p>=.95)strong++;}else if(p<=.15){pattern.push_back('0');stable++;if(p<=.05)strong++;}else pattern.push_back('.');}
        ImGui::Text("Sync-aligned blocks: %u x %d bits",blocks,frameBits);
        ImGui::Text("Stable positions: %d/%d (%.0f%%); very stable: %d",stable,frameBits,100.0*stable/frameBits,strong);
        ImGui::TextUnformatted("Sync-aligned consensus ('.' = variable):");
        for(size_t off=0;off<pattern.size();off+=32){const std::string line=pattern.substr(off,32);ImGui::Text("%3u: %s",(unsigned)off,line.c_str());}
    }
    void drawStabilityMap(const std::vector<uint8_t>& bits,const std::vector<std::pair<size_t,size_t>>& regions,int frameBits){
        if(frameBits<=0)return;
        std::vector<uint32_t> oneCount(frameBits,0),sampleCount(frameBits,0);uint32_t blocks=0;
        for(auto r:regions){
            const size_t length=r.second-r.first;const size_t complete=length/size_t(frameBits);blocks+=(uint32_t)complete;
            for(size_t block=0;block<complete;block++)for(int pos=0;pos<frameBits;pos++){
                const uint8_t bit=bits[r.first+block*size_t(frameBits)+size_t(pos)];oneCount[pos]+=bit;sampleCount[pos]++;
            }
        }
        if(blocks<2)return;
        std::string pattern;pattern.reserve(frameBits);int stable=0,strong=0;
        for(int pos=0;pos<frameBits;pos++){
            const double p=sampleCount[pos]?double(oneCount[pos])/sampleCount[pos]:.5;
            if(p>=.85){pattern.push_back('1');stable++;if(p>=.95)strong++;}
            else if(p<=.15){pattern.push_back('0');stable++;if(p<=.05)strong++;}
            else pattern.push_back('.');
        }
        ImGui::Spacing();ImGui::Text("Aligned blocks: %u x %d bits",blocks,frameBits);
        ImGui::Text("Stable positions: %d/%d (%.0f%%); very stable: %d",stable,frameBits,100.0*stable/frameBits,strong);
        ImGui::TextUnformatted("Consensus ('.' = variable):");
        // Split long patterns so narrow side panels wrap at deterministic bit boundaries.
        for(size_t off=0;off<pattern.size();off+=32){const std::string line=pattern.substr(off,32);ImGui::Text("%3u: %s",(unsigned)off,line.c_str());}
    }
    void startCapture(){
        std::lock_guard<std::mutex> l(captureMutex);if(captureFile.is_open())return;
        std::string folder=core::args["root"].s()+"/recordings";std::filesystem::create_directories(folder);
        std::time_t now=std::time(nullptr);std::tm tm{};localtime_s(&tm,&now);std::ostringstream p;p<<folder<<"/stanag4481_"<<std::put_time(&tm,"%Y%m%d_%H%M%S")<<".bin";capturePath=p.str();
        captureFile.open(capturePath,std::ios::binary|std::ios::trunc);confidenceFile.open(capturePath+".confidence.u8",std::ios::binary|std::ios::trunc);
        frameFile.open(capturePath+".frames113.bin",std::ios::binary|std::ios::trunc);frameConfidenceFile.open(capturePath+".frames113.confidence.u8",std::ios::binary|std::ios::trunc);
        correctedFrameFile.open(capturePath+".frames113.corrected.bin",std::ios::binary|std::ios::trunc);correctionMapFile.open(capturePath+".frames113.corrections.u8",std::ios::binary|std::ios::trunc);
        captureByte=0;captureBitPos=0;capturedBytes=0;capturing.store(captureFile.is_open()&&confidenceFile.is_open()&&frameFile.is_open()&&frameConfidenceFile.is_open()&&correctedFrameFile.is_open()&&correctionMapFile.is_open());
    }
    void stopCapture(){
        std::lock_guard<std::mutex> l(captureMutex);capturing=false;
        if(captureFile.is_open()){if(captureBitPos){captureByte<<=(8-captureBitPos);captureFile.put((char)captureByte);capturedBytes++;}captureFile.close();}
        if(confidenceFile.is_open())confidenceFile.close();
        if(frameFile.is_open())frameFile.close();if(frameConfidenceFile.is_open())frameConfidenceFile.close();
        if(correctedFrameFile.is_open())correctedFrameFile.close();if(correctionMapFile.is_open())correctionMapFile.close();
        captureByte=0;captureBitPos=0;
    }
    void process(const dsp::complex_t& x){
        const double c1=std::cos(markPhase),s1=std::sin(markPhase),c2=std::cos(spacePhase),s2=std::sin(spacePhase);
        constexpr double a=.035;
        markI+=(x.re*c1+x.im*s1-markI)*a; markQ+=(x.im*c1-x.re*s1-markQ)*a;
        spaceI+=(x.re*c2+x.im*s2-spaceI)*a; spaceQ+=(x.im*c2-x.re*s2-spaceQ)*a;
        markPhase+=2*PI*TONE/RATE; spacePhase-=2*PI*TONE/RATE;
        if(markPhase>PI)markPhase-=2*PI;if(spacePhase<-PI)spacePhase+=2*PI;
        const double pm=markI*markI+markQ*markQ, ps=spaceI*spaceI+spaceQ*spaceQ;
        const double instantaneousPower=x.re*x.re+x.im*x.im;
        inputPower+=(instantaneousPower-inputPower)*.002;
        const double concentration=(pm+ps)/(inputPower+1e-15);
        carrierMetric+=(concentration-carrierMetric)*.002;toneConcentration.store((float)carrierMetric);
        const bool evidence=inputPower>1e-12&&carrierMetric>(carrierPresent.load()?.075:.12);
        if(evidence){carrierOffSamples=0;if(carrierOnSamples<400)carrierOnSamples++;if(carrierOnSamples>=400)carrierPresent=true;}
        else{carrierOnSamples=0;if(carrierOffSamples<800)carrierOffSamples++;if(carrierOffSamples>=800)carrierPresent=false;}
        const bool raw=pm>ps; const float q=(float)(std::abs(pm-ps)/(pm+ps+1e-15));
        if(raw!=lastRaw){
            // IQ calibration places the filtered mark/space crossover at
            // phase 0.56 (the ideal 0.5 boundary plus detector delay). Use a
            // gentle phase loop: enough to follow clock drift without making
            // average baud rate depend on transition density.
            constexpr double transitionPhase=.56,timingGain=.04;
            symbolPhase+=timingGain*(transitionPhase-symbolPhase);
            lastRaw=raw;samplesSinceTransition=0;
        }
        else if(samplesSinceTransition<1000)samplesSinceTransition++;
        // The two tone filters necessarily overlap just after a mark/space edge.
        // Exclude that settling interval so displayed confidence describes the
        // stable part of each symbol rather than the number of data transitions.
        if(samplesSinceTransition>=20&&carrierPresent.load())quality.store(quality.load()*.98f+q*.02f);
        else if(!carrierPresent.load())quality.store(quality.load()*.995f);
        symbolPhase+=BAUD/RATE;
        // Always inspect candidate symbols for the sync word, but putBit only
        // exposes/captures them after protocol synchronization is verified.
        if(symbolPhase>=1.0){symbolPhase-=1.0;putBit(raw^reverse.load(),q);}
    }
    void resetSignalLock(){
        frameCollecting=false;frameBits.clear();frameReliability.clear();syncShift=0;syncReliabilityCount=0;syncReliabilityPos=0;
        currentByte=0;bitsInByte=0;havePreviousBit=false;currentRun=0;symbolPhase=0;
    }
    void putBit(bool bit,float confidence){
        candidateBitCount++;
        if(protocolLocked.load()&&candidateBitCount.load()-lastProtocolCandidate.load()>180){protocolLocked=false;resetSignalLock();}
        const uint8_t confidenceByte=(uint8_t)std::clamp<int>((int)std::lround(confidence*255.0f),0,255);
        collectFrame(bit,confidenceByte);
        if(!protocolLocked.load())return;
        bitCount++;if(bit)ones++;oneRatio.store(bitCount?float(ones.load())/float(bitCount.load()):0);
        if(havePreviousBit&&bit!=previousBit)transitionCount++;previousBit=bit;havePreviousBit=true;
        currentRun=(currentRun&&bit==runBit)?currentRun+1:1;runBit=bit;if(currentRun>longestRun.load())longestRun=currentRun;
        {std::lock_guard<std::mutex> l(dataMutex);recentBits.push_back(bit?1:0);recentReliability.push_back(confidenceByte);if(recentBits.size()>4096){recentBits.erase(recentBits.begin(),recentBits.begin()+1024);recentReliability.erase(recentReliability.begin(),recentReliability.begin()+1024);}}
        if(capturing.load()){std::lock_guard<std::mutex> l(captureMutex);if(captureFile.is_open()){captureByte=uint8_t((captureByte<<1)|(bit?1:0));confidenceFile.put((char)confidenceByte);if(++captureBitPos==8){captureFile.put((char)captureByte);capturedBytes++;captureByte=0;captureBitPos=0;}}}
        currentByte=(currentByte<<1)|(bit?1:0);if(++bitsInByte==8){std::lock_guard<std::mutex> l(dataMutex);bytes.push_back(currentByte);if(bytes.size()>512)bytes.erase(bytes.begin(),bytes.begin()+128);currentByte=0;bitsInByte=0;}
    }
    static void handler(dsp::complex_t* data,int count,void* ctx){auto* s=static_cast<Stanag4481Module*>(ctx);for(int i=0;i<count;i++)s->process(data[i]);}

    void collectFrame(bool bit,uint8_t confidence){
        // The capture established that the leading two bits of the original
        // 32-bit candidate are payload-dependent. The lower 30 bits are the
        // stable synchronization core.
        constexpr uint32_t syncWord=0x1DBB7691u,syncMask=0x3FFFFFFFu;syncShift=(syncShift<<1)|(bit?1u:0u);
        syncReliability[syncReliabilityPos]=confidence;syncReliabilityPos=(syncReliabilityPos+1)%32;if(syncReliabilityCount<32)syncReliabilityCount++;
        if(frameCollecting){frameBits.push_back(bit?1:0);frameReliability.push_back(confidence);if(frameBits.size()==113)finishFrame();return;}
        if(syncReliabilityCount<32)return;const int errors=hamming32((syncShift^syncWord)&syncMask);if(errors>2)return;
        int relSum=0;for(int n=2;n<32;n++)relSum+=syncReliability[(syncReliabilityPos+n)%32];if(relSum<30*128){rejectedSync++;return;}
        protocolLocked=true;lastProtocolCandidate=candidateBitCount.load();
        frameBits.clear();frameReliability.clear();frameBits.reserve(113);frameReliability.reserve(113);
        // Begin at the stable 30-bit core, excluding its two variable prefix bits.
        for(int n=29;n>=0;n--)frameBits.push_back((syncShift>>n)&1u);
        for(int n=2;n<32;n++)frameReliability.push_back(syncReliability[(syncReliabilityPos+n)%32]);
        currentSyncErrors=errors;frameCollecting=true;
    }
    void finishFrame(){
        frameCollecting=false;acceptedFrames++;syncErrorTotal+=currentSyncErrors;lastFrameBit=bitCount.load();lastProtocolCandidate=candidateBitCount.load();
        struct LinearCheck{int a,b,c,expected;};
        static constexpr LinearCheck checks[]={
            {73,75,-1,0},{44,47,-1,0},{41,45,-1,0},{37,40,-1,0},{37,38,-1,1},
            {34,38,-1,0},{34,37,-1,1},{31,33,-1,1},{30,33,-1,0},{27,33,-1,1},
            {27,31,-1,0},{13,14,-1,1},{49,52,53,0},{63,67,75,0}
        };
        int evaluated=0,failures=0;
        for(const auto& check:checks){const int a=30+check.a,b=30+check.b,c=check.c<0?-1:30+check.c;
            if(frameReliability[a]<128||frameReliability[b]<128||(c>=0&&frameReliability[c]<128))continue;
            const int parity=frameBits[a]^frameBits[b]^(c>=0?frameBits[c]:0);evaluated++;if(parity!=check.expected)failures++;}
        const bool classifiedFailed=evaluated>=8&&failures>0;
        if(evaluated<8)uncertainFrames++;else if(failures)failedFrames++;else validFrames++;
        auto allFailures=[&](const std::vector<uint8_t>& candidate){int count=0;for(const auto& check:checks){const int a=30+check.a,b=30+check.b,c=check.c<0?-1:30+check.c;const int parity=candidate[a]^candidate[b]^(c>=0?candidate[c]:0);if(parity!=check.expected)count++;}return count;};
        std::vector<uint8_t> corrected=frameBits;uint8_t correctionCode=0;
        if(classifiedFailed&&allFailures(frameBits)>0){int unique=-1;
            for(int payloadPos=0;payloadPos<83;payloadPos++){const int framePos=30+payloadPos;if(frameReliability[framePos]>=160)continue;corrected[framePos]^=1;
                if(allFailures(corrected)==0){if(unique>=0){unique=-2;corrected[framePos]^=1;break;}unique=framePos;}corrected[framePos]^=1;}
            if(unique>=0){corrected=frameBits;corrected[unique]^=1;correctionCode=(uint8_t)(unique-30+1);correctedFrames++;}else{corrected=frameBits;uncorrectableFrames++;}
        }
        // Captures show a provisional 85-frame cycle. Compare only the 83-bit
        // payload; the 30-bit synchronization core is identical by definition.
        {std::lock_guard<std::mutex> l(dataMutex);if(cycleHistory.size()==85){int d=0;for(int n=30;n<113;n++)d+=frameBits[n]!=cycleHistory.front()[n];cycleComparisons++;cycleDistanceTotal+=d;if(d==0)cycleExact++;if(d<=4)cycleNear++;cycleHistory.erase(cycleHistory.begin());}cycleHistory.push_back(frameBits);}
        std::ostringstream text;text<<std::hex<<std::uppercase<<std::setfill('0');uint8_t packed=0;int pos=0;std::vector<uint8_t> packedFrame;
        for(uint8_t bit:frameBits){packed=uint8_t((packed<<1)|bit);if(++pos==8){packedFrame.push_back(packed);text<<std::setw(2)<<(int)packed;packed=0;pos=0;}}
        if(pos){packed<<=(8-pos);packedFrame.push_back(packed);text<<std::setw(2)<<(int)packed;}
        packed=0;pos=0;std::vector<uint8_t> correctedPacked;
        for(uint8_t bit:corrected){packed=uint8_t((packed<<1)|bit);if(++pos==8){correctedPacked.push_back(packed);packed=0;pos=0;}}
        if(pos){packed<<=(8-pos);correctedPacked.push_back(packed);}
        {std::lock_guard<std::mutex> l(dataMutex);lastFrameHex=text.str();}
        if(capturing.load()){std::lock_guard<std::mutex> l(captureMutex);if(frameFile.is_open()){frameFile.write((char*)packedFrame.data(),packedFrame.size());frameConfidenceFile.write((char*)frameReliability.data(),frameReliability.size());correctedFrameFile.write((char*)correctedPacked.data(),correctedPacked.size());correctionMapFile.put((char)correctionCode);}}
    }

    std::string name; bool enabled=true; VFOManager::VFO* vfo=nullptr; dsp::sink::Handler<dsp::complex_t> sink;
    double markPhase=0,spacePhase=0,markI=0,markQ=0,spaceI=0,spaceQ=0,symbolPhase=0,inputPower=0,carrierMetric=0; bool lastRaw=false;int samplesSinceTransition=1000,carrierOnSamples=0,carrierOffSamples=0;
    std::atomic<bool> reverse{false},capturing{false},carrierPresent{false},protocolLocked{false};std::atomic<float> quality{0},oneRatio{0},toneConcentration{0};
    std::atomic<uint64_t> bitCount{0},candidateBitCount{0},lastProtocolCandidate{0},ones{0},transitionCount{0},longestRun{0},capturedBytes{0},acceptedFrames{0},rejectedSync{0},syncErrorTotal{0},lastFrameBit{0},validFrames{0},failedFrames{0},uncertainFrames{0},correctedFrames{0},uncorrectableFrames{0},cycleComparisons{0},cycleExact{0},cycleNear{0},cycleDistanceTotal{0};
    uint64_t currentRun=0;bool runBit=false,previousBit=false,havePreviousBit=false;
    uint8_t currentByte=0,captureByte=0;int bitsInByte=0,captureBitPos=0;
    uint32_t syncShift=0;std::array<uint8_t,32> syncReliability{};int syncReliabilityPos=0,syncReliabilityCount=0,currentSyncErrors=0;bool frameCollecting=false;
    std::vector<uint8_t> frameBits,frameReliability;
    std::mutex dataMutex,captureMutex;std::vector<uint8_t> bytes,recentBits,recentReliability;std::vector<std::vector<uint8_t>> cycleHistory;std::ofstream captureFile,confidenceFile,frameFile,frameConfidenceFile,correctedFrameFile,correctionMapFile;std::string capturePath,lastFrameHex;
};

MOD_EXPORT void _INIT_(){}
MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name){return new Stanag4481Module(std::move(name));}
MOD_EXPORT void _DELETE_INSTANCE_(void* instance){delete static_cast<Stanag4481Module*>(instance);}
MOD_EXPORT void _END_(){}
