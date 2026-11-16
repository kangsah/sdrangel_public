#include "bfmsharedvfobranch.h"

#include <QDebug>

#include "dsp/dspdevicesourceengine.h"
#include "bfmdemodbaseband.h"

QMutex BFMSharedVFOBranch::s_mutex;
QHash<QString, std::weak_ptr<BFMSharedVFOBranch>> BFMSharedVFOBranch::s_branches;

std::shared_ptr<BFMSharedVFOBranch> BFMSharedVFOBranch::acquire(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        DSPDeviceSourceEngine* sourceEngine,
        BFMDemodBaseband* output)
{
    QMutexLocker mutexLocker(&s_mutex);
    std::shared_ptr<BFMSharedVFOBranch> branch;

    auto it = s_branches.find(key);
    if (it != s_branches.end()) {
        branch = it.value().lock();
    }

    if (!branch) {
        branch = std::shared_ptr<BFMSharedVFOBranch>(
            new BFMSharedVFOBranch(
                key,
                inputSampleRate,
                outputSampleRate,
                decimation,
                frequencyOffset,
                sourceEngine,
                output));
        s_branches.insert(key, branch);
        qDebug() << "BFMSharedVFOBranch::acquire: created key:" << key;
    } else {
        branch->worker()->addOutput(output);
        qDebug() << "BFMSharedVFOBranch::acquire: reused key:" << key;
    }

    return branch;
}

BFMSharedVFOBranch::BFMSharedVFOBranch(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        DSPDeviceSourceEngine* sourceEngine,
        BFMDemodBaseband* output) :
    m_thread(new QThread()),
    m_worker(new BFMEarlyIFWorker()),
    m_key(key),
    m_sourceEngine(sourceEngine),
    m_haveLastBlock(false)
{
    m_thread->setObjectName("BFMSharedVFO");
    m_worker->setOutput(output);
    m_worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);

    m_worker->configure(inputSampleRate, outputSampleRate, decimation, frequencyOffset);
    m_worker->startWork();
    m_thread->start();
    m_sourceEngine->addSink(this);
}

BFMSharedVFOBranch::~BFMSharedVFOBranch()
{
    m_sourceEngine->removeSink(this, true);
    m_worker->stopWork();
    m_thread->quit();
    m_thread->wait();
    m_worker = nullptr;
    m_thread = nullptr;
}

void BFMSharedVFOBranch::feed(
        const SampleVector::const_iterator& begin,
        const SampleVector::const_iterator& end,
        bool positiveOnly)
{
    (void) positiveOnly;
    m_worker->feed(begin, end);
}

void BFMSharedVFOBranch::feed(
        const SampleVector::const_iterator& begin,
        const SampleVector::const_iterator& end)
{
    QMutexLocker mutexLocker(&m_feedMutex);

    if (m_haveLastBlock && (begin == m_lastBegin) && (end == m_lastEnd)) {
        return;
    }

    m_lastBegin = begin;
    m_lastEnd = end;
    m_haveLastBlock = true;
    m_worker->feed(begin, end);
}

void BFMSharedVFOBranch::configure(
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset)
{
    m_worker->configure(inputSampleRate, outputSampleRate, decimation, frequencyOffset);
}

void BFMSharedVFOBranch::removeOutput(BFMDemodBaseband* output)
{
    m_worker->removeOutput(output);
}
