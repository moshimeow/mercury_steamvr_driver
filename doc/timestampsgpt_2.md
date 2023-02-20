#include <dshow.h>
#include <iostream>

int main() {
    CoInitialize(NULL);

    IGraphBuilder* graphBuilder = NULL;
    IMediaControl* mediaControl = NULL;
    IBaseFilter* captureFilter = NULL;
    IBaseFilter* sampleGrabberFilter = NULL;
    ISampleGrabber* sampleGrabber = NULL;

    HRESULT hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void**)&graphBuilder);
    hr = graphBuilder->QueryInterface(IID_IMediaControl, (void**)&mediaControl);

    hr = CoCreateInstance(CLSID_VideoInputDeviceCategory, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (void**)&captureFilter);
    hr = graphBuilder->AddFilter(captureFilter, L"Capture Filter");

    hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (void**)&sampleGrabberFilter);
    hr = graphBuilder->AddFilter(sampleGrabberFilter, L"Sample Grabber");

    hr = sampleGrabberFilter->QueryInterface(IID_ISampleGrabber, (void**)&sampleGrabber);
    AM_MEDIA_TYPE mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
    mt.majortype = MEDIATYPE_Video;
    mt.subtype = MEDIASUBTYPE_RGB24;
    hr = sampleGrabber->SetMediaType(&mt);
    hr = sampleGrabber->SetCallback(NULL, 0);

    IPin* captureOutPin = NULL;
    hr = captureFilter->FindPin(L"Capture", &captureOutPin);
    IPin* grabberInPin = NULL;
    hr = sampleGrabberFilter->FindPin(L"In", &grabberInPin);

    hr = graphBuilder->ConnectDirect(captureOutPin, grabberInPin, &mt);

    IMediaEvent* mediaEvent = NULL;
    hr = graphBuilder->QueryInterface(IID_IMediaEvent, (void**)&mediaEvent);

    hr = mediaControl->Run();

    long eventCode;
    do {
        mediaEvent->WaitForCompletion(100, &eventCode);
        if (eventCode == EC_SAMPLE_COMPLETED) {
            IMediaSample* mediaSample = NULL;
            hr = sampleGrabber->GetCurrentBuffer(NULL, &mediaSample);
            if (SUCCEEDED(hr)) {
                REFERENCE_TIME sampleTime;
                hr = mediaSample->GetTime(&sampleTime, NULL);
                if (SUCCEEDED(hr)) {
                    std::cout << "Sample time: " << sampleTime << std::endl;
                }
                mediaSample->Release();
            }
        }
    } while (eventCode != EC_USERABORTED);


    mediaControl->Stop();

    mediaEvent->Release();
    sampleGrabber->Release();
    sampleGrabberFilter->Release();
    captureFilter->Release();
    mediaControl->Release();
    graphBuilder->Release();

    CoUninitialize();

    return 0;
}