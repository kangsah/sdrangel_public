#include "bfmsharedvfobranch.h"

#include "bfmdemodbaseband.h"

QMutex BFMSharedVFOBranch::s_mutex;
QHash<QString, std::weak_ptr<BFMSharedVFOBranch>> BFMSharedVFOBranch::s_branches;

std::shared_ptr<BFMSharedVFOBranch> BFMSharedVFOBranch::acquire(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
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
                output));
        s_branches.insert(key, branch);
    } else {
        branch->worker()->addOutput(output);
    }

    return branch;
}

BFMSharedVFOBranch::BFMSharedVFOBranch(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        BFMDemodBaseband* output) :
    m_thread(new QThread()),
    m_worker(new BFMEarlyIFWorker()),
    m_key(key)
{
    m_thread->setObjectName("BFMSharedVFO");
    m_worker->setOutput(output);
    m_worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);

    m_worker->configure(inputSampleRate, outputSampleRate, decimation, frequencyOffset);
    m_worker->startWork();
    m_thread->start();
}

BFMSharedVFOBranch::~BFMSharedVFOBranch()
{
    m_worker->stopWork();
    m_thread->quit();
    m_thread->wait();
    m_worker = nullptr;
    m_thread = nullptr;
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
