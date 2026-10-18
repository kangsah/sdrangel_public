///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 SDRangel contributors                                      //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                    //
///////////////////////////////////////////////////////////////////////////////////

#ifndef PLUGINS_CHANNELRX_DEMODBFM_BFMCAPPEDIF_H_
#define PLUGINS_CHANNELRX_DEMODBFM_BFMCAPPEDIF_H_

#include "dsp/dsptypes.h"
#include "dsp/interpolator.h"
#include "dsp/nco.h"

class BFMDemodSink;

class BFMCappedIF
{
public:
    static constexpr int CappedSampleRate = 250000;

    explicit BFMCappedIF(BFMDemodSink* sink);

    void configure(int inputSampleRate, qint64 frequencyOffset, int rfBandwidth);
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    int getSampleRate() const { return m_outputSampleRate; }

private:
    BFMDemodSink* m_sink;
    NCO m_nco;
    Interpolator m_resampler;
    Real m_distance;
    Real m_distanceRemain;
    int m_inputSampleRate;
    int m_outputSampleRate;
    SampleVector m_outputBuffer;
};

#endif // PLUGINS_CHANNELRX_DEMODBFM_BFMCAPPEDIF_H_
