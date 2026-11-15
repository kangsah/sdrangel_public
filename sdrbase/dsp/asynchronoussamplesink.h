///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026                                                          //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by the     //
// Free Software Foundation; either version 3 of the License, or               //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_ASYNCHRONOUSSAMPLESINK_H
#define INCLUDE_ASYNCHRONOUSSAMPLESINK_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "dsp/basebandsamplesink.h"
#include "dsp/samplesinkfifo.h"

class SDRBASE_API AsynchronousSampleSink : public BasebandSampleSink
{
public:
    explicit AsynchronousSampleSink(BasebandSampleSink *sink);
    ~AsynchronousSampleSink() override;

    void start() override;
    void stop() override;
    void stopWithoutTarget();
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool positiveOnly) override;
    void pushMessage(Message *msg) override;
    QString getSinkName() override;

private:
    BasebandSampleSink *m_sink;
    SampleSinkFifo m_fifo;
    std::thread m_thread;
    std::mutex m_waitMutex;
    std::condition_variable m_waitCondition;
    std::atomic<bool> m_running;
    std::atomic<bool> m_stopRequested;
    std::atomic<bool> m_positiveOnly;

    void process();
    void stopInternal(bool stopTarget);
};

#endif // INCLUDE_ASYNCHRONOUSSAMPLESINK_H
