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
    m_intermediateSampleRate(0),
    m_powerOfTwoDecimation(1),
    m_valid(false),
    m_inputSamples(0),
    m_decimatedSamples(0),
    m_resamplerInputSamples(0),
    m_outputSamples(0),
    m_nextReportInputSamples(1000000)
{
}

void BFMCappedIF::configure(int inputSampleRate, qint64 frequencyOffset, int rfBandwidth)
{
    m_inputSampleRate = inputSampleRate;
    m_outputSampleRate = std::min(CappedSampleRate, inputSampleRate);
    m_intermediateSampleRate = 0;
    m_powerOfTwoDecimation = 1;
    m_valid = false;
    m_decimator1.reset();
    m_decimator2.reset();
    m_inputSamples = 0;
    m_decimatedSamples = 0;
    m_resamplerInputSamples = 0;
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

    while ((m_powerOfTwoDecimation < 128)
        && (m_inputSampleRate / static_cast<int>(m_powerOfTwoDecimation * 2) >= m_outputSampleRate))
    {
        m_powerOfTwoDecimation *= 2;
    }

    m_intermediateSampleRate = m_inputSampleRate / static_cast<int>(m_powerOfTwoDecimation);

    m_distance = static_cast<Real>(m_intermediateSampleRate) / m_outputSampleRate;

    if ((m_distance < static_cast<Real>(1.0)) || (m_distance >= static_cast<Real>(2.0)))
    {
        qWarning() << "BFMCappedIF::configure: unsupported final ratio"
                   << "inputRate:" << m_inputSampleRate
                   << "outputRate:" << m_outputSampleRate
                   << "decimation:" << m_powerOfTwoDecimation
                   << "intermediateRate:" << m_intermediateSampleRate
                   << "finalInputPerOutput:" << m_distance;
        return;
    }

    const Real outputNyquist = static_cast<Real>(m_outputSampleRate) / static_cast<Real>(2.0);
    const Real maximumCutoff = outputNyquist - static_cast<Real>(5000.0);

    if ((rfBandwidth <= 0)
        || (maximumCutoff <= static_cast<Real>(1000.0))
        || (static_cast<Real>(rfBandwidth) / static_cast<Real>(2.0) > maximumCutoff))
    {
        qWarning() << "BFMCappedIF::configure: RF bandwidth does not fit capped IF"
                   << "rfBandwidth:" << rfBandwidth
                   << "outputRate:" << m_outputSampleRate
                   << "maximumCutoff:" << maximumCutoff;
        return;
    }

    if (m_powerOfTwoDecimation > 1)
    {
        if (m_powerOfTwoDecimation <= 64)
        {
            m_decimator1 = std::make_unique<DecimatorC>();
            unsigned int log2Decimation = 0;
            unsigned int decimation = m_powerOfTwoDecimation;

            while (decimation > 1)
            {
                ++log2Decimation;
                decimation >>= 1;
            }

            m_decimator1->setLog2Decim(log2Decimation);
        }
        else
        {
            m_decimator1 = std::make_unique<DecimatorC>();
            m_decimator1->setLog2Decim(6);
            m_decimator2 = std::make_unique<DecimatorC>();
            m_decimator2->setLog2Decim(1);
        }
    }

    // Keep the complete configured RF channel below the capped IF Nyquist rate.
    const Real cutoff = std::max(
        static_cast<Real>(1000.0),
        std::min(
            static_cast<Real>(rfBandwidth) / static_cast<Real>(2.0),
            static_cast<Real>(m_outputSampleRate) / static_cast<Real>(2.0) - static_cast<Real>(5000.0)));
    m_resampler.create(64, m_intermediateSampleRate, cutoff);
    m_distanceRemain = m_distance;
    m_outputBuffer.clear();
    m_valid = true;

    qDebug() << "BFMCappedIF::configure:"
             << "inputRate:" << m_inputSampleRate
             << "outputRate:" << m_outputSampleRate
             << "powerOfTwoDecimation:" << m_powerOfTwoDecimation
             << "intermediateRate:" << m_intermediateSampleRate
             << "finalInputPerOutput:" << m_distance
             << "frequencyOffset:" << frequencyOffset
             << "rfBandwidth:" << rfBandwidth
             << "cutoff:" << cutoff;
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
        Complex stage1;
        Complex stage2;
        Complex finalInput = translated;

        if (m_decimator1 && !m_decimator1->decimate(finalInput, stage1)) {
            continue;
        }

        if (m_decimator1) {
            finalInput = stage1;
            ++m_decimatedSamples;
        }

        if (m_decimator2 && !m_decimator2->decimate(finalInput, stage2)) {
            continue;
        }

        if (m_decimator2) {
            finalInput = stage2;
        }

        Complex output;
        ++m_resamplerInputSamples;

        if (m_resampler.decimate(&m_distanceRemain, finalInput, &output))
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
                 << "decimatedSamples:" << m_decimatedSamples
                 << "resamplerInputSamples:" << m_resamplerInputSamples
                 << "outputSamples:" << m_outputSamples
                 << "expectedOutputSamples:" << expectedOutputSamples
                 << "inputRate:" << m_inputSampleRate
                 << "outputRate:" << m_outputSampleRate;
        m_nextReportInputSamples =
            ((m_inputSamples / 1000000) + 1) * 1000000;
    }
}
