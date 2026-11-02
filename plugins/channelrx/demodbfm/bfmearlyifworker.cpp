///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026                                                          //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation; either version 3 of the License, or             //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <QDebug>

#include "dsp/dsptypes.h"

#include "bfmdemodbaseband.h"
#include "bfmearlyifworker.h"

BFMEarlyIFWorker::BFMEarlyIFWorker() :
    m_output(nullptr),
    m_inputSampleRate(0),
    m_outputSampleRate(0),
    m_decimation(1),
    m_inputSamples(0),
    m_outputSamples(0),
    m_droppedInputSamples(0),
    m_active(false),
    m_running(false)
{
    m_inputFifo.setLabel("BFMEarlyIFInput");
    m_inputFifo.setSize(SampleSinkFifo::getSizePolicy(48000));
}

BFMEarlyIFWorker::~BFMEarlyIFWorker()
{
    stopWork();
}

void BFMEarlyIFWorker::configure(
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset)
{
    QMutexLocker mutexLocker(&m_mutex);

    m_inputFifo.reset();
    m_inputFifo.setSize(SampleSinkFifo::getSizePolicy(std::max(inputSampleRate, 48000)));
    m_inputSampleRate = inputSampleRate;
    m_outputSampleRate = outputSampleRate;
    m_decimation = decimation;
    m_inputSamples = 0;
    m_outputSamples = 0;
    m_droppedInputSamples = 0;
    m_outputBuffer.clear();
    m_decimator1.reset();
    m_decimator2.reset();
    m_active = inputSampleRate > 0 && outputSampleRate > 0 && decimation > 1;

    if (!m_active) {
        return;
    }

    m_nco.setFreq(-frequencyOffset, inputSampleRate);
    m_nco.setPhase(0);

    if (decimation <= 64) {
        m_decimator1 = std::make_unique<DecimatorC>();
        unsigned int log2Decimation = 0;
        unsigned int stageDecimation = decimation;

        while (stageDecimation > 1) {
            ++log2Decimation;
            stageDecimation >>= 1;
        }

        m_decimator1->setLog2Decim(log2Decimation);
    } else {
        m_decimator1 = std::make_unique<DecimatorC>();
        m_decimator1->setLog2Decim(6);
        m_decimator2 = std::make_unique<DecimatorC>();
        m_decimator2->setLog2Decim(1);
    }

    qDebug() << "BFMEarlyIFWorker::configure:"
             << "inputRate:" << m_inputSampleRate
             << "outputRate:" << m_outputSampleRate
             << "decimation:" << m_decimation
             << "frequencyOffset:" << frequencyOffset
             << "fifoSize:" << m_inputFifo.size();
}

void BFMEarlyIFWorker::reset()
{
    QMutexLocker mutexLocker(&m_mutex);
    m_inputFifo.reset();
    m_inputSamples = 0;
    m_outputSamples = 0;
    m_droppedInputSamples = 0;
    m_outputBuffer.clear();
}

void BFMEarlyIFWorker::startWork()
{
    QMutexLocker mutexLocker(&m_mutex);

    if (m_running) {
        return;
    }

    QObject::connect(
        &m_inputFifo,
        &SampleSinkFifo::dataReady,
        this,
        &BFMEarlyIFWorker::handleData,
        Qt::QueuedConnection
    );
    m_running = true;
}

void BFMEarlyIFWorker::stopWork()
{
    QMutexLocker mutexLocker(&m_mutex);

    if (!m_running) {
        return;
    }

    QObject::disconnect(
        &m_inputFifo,
        &SampleSinkFifo::dataReady,
        this,
        &BFMEarlyIFWorker::handleData
    );
    m_running = false;
}

void BFMEarlyIFWorker::feed(
        const SampleVector::const_iterator& begin,
        const SampleVector::const_iterator& end)
{
    const unsigned int count = end - begin;
    const unsigned int written = m_inputFifo.write(begin, end);

    if (written < count) {
        QMutexLocker mutexLocker(&m_mutex);
        m_droppedInputSamples += count - written;
    }
}

void BFMEarlyIFWorker::handleData()
{
    process();
}

void BFMEarlyIFWorker::process()
{
    QMutexLocker mutexLocker(&m_mutex);

    while (m_inputFifo.fill() > 0) {
        SampleVector::iterator part1begin;
        SampleVector::iterator part1end;
        SampleVector::iterator part2begin;
        SampleVector::iterator part2end;
        const unsigned int count = m_inputFifo.readBegin(
            m_inputFifo.fill(),
            &part1begin,
            &part1end,
            &part2begin,
            &part2end);

        processPart(part1begin, part1end);
        processPart(part2begin, part2end);
        m_inputFifo.readCommit(count);
    }

    if (m_inputSamples >= 1000000) {
        qDebug() << "BFMEarlyIFWorker::process:"
                 << "inputSamples:" << m_inputSamples
                 << "outputSamples:" << m_outputSamples
                 << "expectedOutputSamples:"
                 << static_cast<double>(m_inputSamples) / m_decimation
                 << "inputRate:" << m_inputSampleRate
                 << "outputRate:" << m_outputSampleRate
                 << "inputFifoFill:" << m_inputFifo.fill()
                 << "inputFifoSize:" << m_inputFifo.size()
                 << "droppedInputSamples:" << m_droppedInputSamples;
        m_inputSamples = 0;
        m_outputSamples = 0;
    }
}

void BFMEarlyIFWorker::processPart(
        SampleVector::iterator begin,
        SampleVector::iterator end)
{
    if (!m_active || !m_output) {
        return;
    }

    const quint64 inputBlockSamples = end - begin;
    m_outputBuffer.clear();
    m_outputBuffer.reserve(static_cast<int>(inputBlockSamples / m_decimation) + 2);

    for (auto it = begin; it != end; ++it) {
        const Complex input(
            it->real() / SDR_RX_SCALEF,
            it->imag() / SDR_RX_SCALEF);
        const Complex translated = input * m_nco.nextIQ();
        Complex stage1;
        Complex stage2;
        Complex output = translated;

        if (m_decimator1 && !m_decimator1->decimate(output, stage1)) {
            continue;
        }
        if (m_decimator1) {
            output = stage1;
        }
        if (m_decimator2 && !m_decimator2->decimate(output, stage2)) {
            continue;
        }
        if (m_decimator2) {
            output = stage2;
        }

        m_outputBuffer.emplace_back(
            output.real() * SDR_RX_SCALEF,
            output.imag() * SDR_RX_SCALEF);
        ++m_outputSamples;
    }

    m_inputSamples += inputBlockSamples;

    if (!m_outputBuffer.empty()) {
        m_output->feed(m_outputBuffer.cbegin(), m_outputBuffer.cend());
    }
}
