///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 SDRangel contributors                                      //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation; version 3 of the License.                    //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <QDebug>

#include "bfmcappedif.h"
#include "bfmdemodsink.h"

BFMCappedIF::BFMCappedIF(BFMDemodSink* sink) :
    m_sink(sink),
    m_distance(1.0),
    m_distanceRemain(1.0),
    m_inputSampleRate(0),
    m_outputSampleRate(0),
    m_inputSamples(0),
    m_outputSamples(0),
    m_nextReportInputSamples(1000000)
{
}

void BFMCappedIF::configure(int inputSampleRate, qint64 frequencyOffset, int rfBandwidth)
{
    m_inputSampleRate = inputSampleRate;
    m_outputSampleRate = std::min(CappedSampleRate, inputSampleRate);
    m_inputSamples = 0;
    m_outputSamples = 0;
    m_nextReportInputSamples = 1000000;

    if ((m_inputSampleRate <= 0) || (m_outputSampleRate <= 0)) {
        m_distance = 1.0;
        m_distanceRemain = 1.0;
        m_outputBuffer.clear();
        qWarning() << "BFMCappedIF::configure: invalid rates"
                   << "input:" << m_inputSampleRate
                   << "output:" << m_outputSampleRate;
        return;
    }

    m_nco.setFreq(-frequencyOffset, m_inputSampleRate);
    m_nco.setPhase(0);

    // Keep the complete configured RF channel below the capped IF Nyquist rate.
    const Real cutoff = std::max(
        static_cast<Real>(1000.0),
        std::min(
            static_cast<Real>(rfBandwidth) / static_cast<Real>(2.0),
            static_cast<Real>(m_outputSampleRate) / static_cast<Real>(2.0) - static_cast<Real>(5000.0)));
    m_resampler.create(64, m_inputSampleRate, cutoff);
    m_distance = static_cast<Real>(m_outputSampleRate) / m_inputSampleRate;
    m_distanceRemain = m_distance;
    m_outputBuffer.clear();

    qDebug() << "BFMCappedIF::configure:"
             << "inputRate:" << m_inputSampleRate
             << "outputRate:" << m_outputSampleRate
             << "frequencyOffset:" << frequencyOffset
             << "rfBandwidth:" << rfBandwidth
             << "cutoff:" << cutoff
             << "prototypeOutputPerInput:" << m_distance
             << "conceptualInputPerOutput:"
             << static_cast<Real>(m_inputSampleRate) / m_outputSampleRate;
}

void BFMCappedIF::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    if ((m_inputSampleRate <= 0) || (m_outputSampleRate <= 0)) {
        return;
    }

    const quint64 inputBlockSamples = static_cast<quint64>(end - begin);
    m_inputSamples += inputBlockSamples;
    m_outputBuffer.clear();
    m_outputBuffer.reserve(static_cast<int>(inputBlockSamples * m_distance) + 2);

    for (auto it = begin; it != end; ++it)
    {
        const Complex input(
            it->real() / SDR_RX_SCALEF,
            it->imag() / SDR_RX_SCALEF);
        const Complex translated = input * m_nco.nextIQ();
        Complex output;

        if (m_resampler.decimate(&m_distanceRemain, translated, &output))
        {
            m_outputBuffer.emplace_back(
                output.real() * SDR_RX_SCALEF,
                output.imag() * SDR_RX_SCALEF);
            ++m_outputSamples;
            m_distanceRemain += m_distance;
        }
    }

    if (!m_outputBuffer.empty()) {
        m_sink->feed(m_outputBuffer.cbegin(), m_outputBuffer.cend());
    }

    if (m_inputSamples >= m_nextReportInputSamples) {
        const double expectedOutputSamples =
            static_cast<double>(m_inputSamples) * m_outputSampleRate / m_inputSampleRate;
        qDebug() << "BFMCappedIF::feed:"
                 << "inputSamples:" << m_inputSamples
                 << "outputSamples:" << m_outputSamples
                 << "expectedOutputSamples:" << expectedOutputSamples
                 << "inputRate:" << m_inputSampleRate
                 << "outputRate:" << m_outputSampleRate;
        m_nextReportInputSamples =
            ((m_inputSamples / 1000000) + 1) * 1000000;
    }
}
