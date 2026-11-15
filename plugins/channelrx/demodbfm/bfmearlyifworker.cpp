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
    m_processingNsecs(0),
    m_processCalls(0),
    m_droppedInputSamples(0),
    m_inputFifoHighWater(0),
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
    m_processingNsecs = 0;
    m_processCalls = 0;
    m_droppedInputSamples.store(0);
    m_inputFifoHighWater.store(0);
    m_decimatorBuffer.clear();
    m_outputComplexBuffer.clear();
    m_outputBuffer.clear();
    m_reportTimer.restart();
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
    m_processingNsecs = 0;
    m_processCalls = 0;
    m_droppedInputSamples.store(0);
    m_inputFifoHighWater.store(0);
    m_decimatorBuffer.clear();
    m_outputComplexBuffer.clear();
    m_outputBuffer.clear();
    m_reportTimer.restart();
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
    const unsigned int fill = m_inputFifo.fill();

    unsigned int highWater = m_inputFifoHighWater.load();
    while ((fill > highWater)
        && !m_inputFifoHighWater.compare_exchange_weak(highWater, fill))
    {
    }
    if (written < count) {
        m_droppedInputSamples.fetch_add(count - written);
    }
}

void BFMEarlyIFWorker::handleData()
{
    process();
}

void BFMEarlyIFWorker::process()
{
    QMutexLocker mutexLocker(&m_mutex);
    QElapsedTimer processTimer;
    processTimer.start();

    const unsigned int availableSamples = m_inputFifo.fill();
    const unsigned int processSamples = std::min(availableSamples, m_maxProcessSamples);

    if (processSamples > 0) {
        SampleVector::iterator part1begin;
        SampleVector::iterator part1end;
        SampleVector::iterator part2begin;
        SampleVector::iterator part2end;
        const unsigned int count = m_inputFifo.readBegin(
            processSamples,
            &part1begin,
            &part1end,
            &part2begin,
            &part2end);

        m_outputBuffer.clear();
        m_outputBuffer.reserve(static_cast<int>(count / m_decimation) + 2);
        processPart(part1begin, part1end);
        processPart(part2begin, part2end);
        m_inputFifo.readCommit(count);

        if (m_output && !m_outputBuffer.empty()) {
            m_output->feed(m_outputBuffer.cbegin(), m_outputBuffer.cend());
        }
    }

    m_processingNsecs += processTimer.nsecsElapsed();
    ++m_processCalls;

    if ((m_reportTimer.elapsed() >= 2000) && (m_inputSamples > 0)) {
        const qint64 reportTimeMsecs = std::max<qint64>(m_reportTimer.elapsed(), 1);
        const double processingTimeSeconds =
            static_cast<double>(m_processingNsecs) / 1000000000.0;
        const double processingRate = processingTimeSeconds > 0.0 ?
            static_cast<double>(m_inputSamples) / processingTimeSeconds : 0.0;
        const double utilizationPercent =
            100.0 * static_cast<double>(m_processingNsecs) /
            (static_cast<double>(reportTimeMsecs) * 1000000.0);

        qDebug() << "BFMEarlyIFWorker::process:"
                 << "inputSamples:" << m_inputSamples
                 << "outputSamples:" << m_outputSamples
                 << "expectedOutputSamples:"
                 << static_cast<double>(m_inputSamples) / m_decimation
                 << "inputRate:" << m_inputSampleRate
                 << "outputRate:" << m_outputSampleRate
                 << "inputFifoFill:" << m_inputFifo.fill()
                 << "inputFifoSize:" << m_inputFifo.size()
                 << "inputFifoHighWater:" << m_inputFifoHighWater.load()
                 << "droppedInputSamples:" << m_droppedInputSamples.load()
                 << "processingTimeMs:" << static_cast<double>(m_processingNsecs) / 1000000.0
                 << "processingRate:" << processingRate
                 << "processingUtilizationPercent:" << utilizationPercent
                 << "processCalls:" << m_processCalls
                 << "processBlockSamples:" << m_maxProcessSamples;
        m_inputSamples = 0;
        m_outputSamples = 0;
        m_processingNsecs = 0;
        m_processCalls = 0;
        m_reportTimer.restart();
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
    const unsigned int inputSamples = static_cast<unsigned int>(inputBlockSamples);

    if (inputSamples == 0) {
        return;
    }

    const unsigned int firstOutputCapacity =
        inputSamples / m_decimator1->getDecim() + 2;
    m_decimatorBuffer.resize(firstOutputCapacity);
    const unsigned int firstOutputCount = m_decimator1->decimateBlockNCO(
        &begin[0],
        inputSamples,
        m_nco,
        m_decimatorBuffer.data());

    const Complex* outputBegin = m_decimatorBuffer.data();
    unsigned int outputCount = firstOutputCount;

    if (m_decimator2) {
        m_outputComplexBuffer.resize(firstOutputCount / m_decimator2->getDecim() + 2);
        outputCount = m_decimator2->decimateBlock(
            m_decimatorBuffer.data(),
            firstOutputCount,
            m_outputComplexBuffer.data());
        outputBegin = m_outputComplexBuffer.data();
    }

    for (unsigned int i = 0; i < outputCount; ++i) {
        m_outputBuffer.emplace_back(
            outputBegin[i].real() * SDR_RX_SCALEF,
            outputBegin[i].imag() * SDR_RX_SCALEF);
    }

    m_inputSamples += inputBlockSamples;
    m_outputSamples += outputCount;
}
