///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026                                                          //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation; either version 3 of the License, or             //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_BFMEARLYIFWORKER_H
#define INCLUDE_BFMEARLYIFWORKER_H

#include <memory>

#include <QMutex>
#include <QObject>

#include "dsp/decimatorc.h"
#include "dsp/nco.h"
#include "dsp/samplesinkfifo.h"

class BFMDemodBaseband;

class BFMEarlyIFWorker : public QObject
{
    Q_OBJECT

public:
    BFMEarlyIFWorker();
    ~BFMEarlyIFWorker() override;

    void setOutput(BFMDemodBaseband *output) { m_output = output; }
    void configure(int inputSampleRate, int outputSampleRate, unsigned int decimation, qint64 frequencyOffset);
    void reset();
    void startWork();
    void stopWork();
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    unsigned int getInputFifoFill() const { return m_inputFifo.fill(); }
    unsigned int getInputFifoSize() const { return m_inputFifo.size(); }

private:
    SampleSinkFifo m_inputFifo;
    BFMDemodBaseband *m_output;
    int m_inputSampleRate;
    int m_outputSampleRate;
    unsigned int m_decimation;
    NCO m_nco;
    std::unique_ptr<DecimatorC> m_decimator1;
    std::unique_ptr<DecimatorC> m_decimator2;
    SampleVector m_outputBuffer;
    quint64 m_inputSamples;
    quint64 m_outputSamples;
    quint64 m_droppedInputSamples;
    QMutex m_mutex;
    bool m_active;
    bool m_running;

    void process();
    void processPart(SampleVector::iterator begin, SampleVector::iterator end);

private slots:
    void handleData();
};

#endif // INCLUDE_BFMEARLYIFWORKER_H
