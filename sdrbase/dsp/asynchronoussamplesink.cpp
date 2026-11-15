///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026                                                          //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by the     //
// the Free Software Foundation; either version 3 of the License, or           //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "dsp/dspcommands.h"

#include <algorithm>

#include "asynchronoussamplesink.h"

AsynchronousSampleSink::AsynchronousSampleSink(BasebandSampleSink *sink) :
    m_sink(sink),
    m_running(false),
    m_stopRequested(false),
    m_positiveOnly(false)
{
    m_fifo.setLabel("AsynchronousSampleSink");
    m_fifo.setSize(SampleSinkFifo::getSizePolicy(48000));
}

AsynchronousSampleSink::~AsynchronousSampleSink()
{
    stopInternal(false);
}

void AsynchronousSampleSink::start()
{
    if (m_running.exchange(true)) {
        return;
    }

    m_stopRequested.store(false);
    m_sink->start();
    m_thread = std::thread(&AsynchronousSampleSink::process, this);
}

void AsynchronousSampleSink::stop()
{
    stopInternal(true);
}

void AsynchronousSampleSink::stopWithoutTarget()
{
    stopInternal(false);
}

void AsynchronousSampleSink::stopInternal(bool stopTarget)
{
    if (!m_running.exchange(false)) {
        if (m_thread.joinable()) {
            m_stopRequested.store(true);
            m_waitCondition.notify_one();
            m_thread.join();
        }
        return;
    }

    m_stopRequested.store(true);
    m_waitCondition.notify_one();

    if (m_thread.joinable()) {
        m_thread.join();
    }

    if (stopTarget && m_sink) {
        m_sink->stop();
    }
}

void AsynchronousSampleSink::feed(
        const SampleVector::const_iterator& begin,
        const SampleVector::const_iterator& end,
        bool positiveOnly)
{
    m_positiveOnly.store(positiveOnly);
    m_fifo.write(begin, end);
    m_waitCondition.notify_one();
}

void AsynchronousSampleSink::pushMessage(Message *msg)
{
    if (DSPSignalNotification::match(*msg)) {
        const DSPSignalNotification& notification = static_cast<const DSPSignalNotification&>(*msg);
        m_fifo.setSize(SampleSinkFifo::getSizePolicy(notification.getSampleRate()));
        m_fifo.reset();
    }

    m_sink->pushMessage(msg);
}

QString AsynchronousSampleSink::getSinkName()
{
    return m_sink->getSinkName();
}

void AsynchronousSampleSink::process()
{
    while (!m_stopRequested.load()) {
        {
            std::unique_lock<std::mutex> lock(m_waitMutex);
            m_waitCondition.wait(lock, [this]() {
                return m_stopRequested.load() || (m_fifo.fill() > 0);
            });
        }

        while (m_fifo.fill() > 0) {
            SampleVector::iterator part1begin;
            SampleVector::iterator part1end;
            SampleVector::iterator part2begin;
            SampleVector::iterator part2end;
            const unsigned int count = m_fifo.readBegin(
                std::min(m_fifo.fill(), 1U << 20),
                &part1begin,
                &part1end,
                &part2begin,
                &part2end);

            if (part1begin != part1end) {
                m_sink->feed(part1begin, part1end, m_positiveOnly.load());
            }
            if (part2begin != part2end) {
                m_sink->feed(part2begin, part2end, m_positiveOnly.load());
            }

            m_fifo.readCommit(count);

            if (m_stopRequested.load()) {
                break;
            }
        }
    }
}
