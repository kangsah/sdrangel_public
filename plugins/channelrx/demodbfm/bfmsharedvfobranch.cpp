#include "bfmsharedvfobranch.h"

#include <QDebug>
#include <algorithm>

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
    m_directDecimation((frequencyOffset == 0) && (decimation > 1)),
    m_decimation(decimation),
    m_directInputSamples(0),
    m_directOutputSamples(0),
    m_directProcessingNsecs(0),
    m_directReportTimer(),
    m_haveLastBlock(false),
    m_preStop(false),
    m_preDroppedSamples(0),
    m_preHighWater(0)
{
    m_thread->setObjectName("BFMSharedVFO");
    m_rawFifo.setLabel("BFMPreDecimatorInput");
    m_rawFifo.setSize(SampleSinkFifo::getSizePolicy(inputSampleRate));
    m_worker->setOutput(output);
    m_worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);

    if (m_directDecimation) {
        m_worker->configure(outputSampleRate, outputSampleRate, 1, 0);
    } else {
        m_worker->configure(inputSampleRate, outputSampleRate, decimation, frequencyOffset);
    }
    m_worker->startWork();
    m_thread->start();
    if (m_directDecimation) {
        m_preThread = std::thread(&BFMSharedVFOBranch::processDirect, this);
    }
    m_sourceEngine->addSink(this);
}

BFMSharedVFOBranch::~BFMSharedVFOBranch()
{
    m_sourceEngine->removeSink(this, true);
    m_preStop.store(true);
    m_preWaitCondition.notify_one();
    if (m_preThread.joinable()) {
        m_preThread.join();
    }
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
    QMutexLocker mutexLocker(&m_feedMutex);

    if (m_haveLastBlock && (begin == m_lastBegin) && (end == m_lastEnd)) {
        return;
    }

    m_lastBegin = begin;
    m_lastEnd = end;
    m_haveLastBlock = true;

    if (!m_directDecimation) {
        m_worker->feed(begin, end);
        return;
    }

    const unsigned int inputSamples = static_cast<unsigned int>(end - begin);
    const unsigned int written = m_rawFifo.write(begin, end);
    const unsigned int fill = m_rawFifo.fill();
    unsigned int highWater = m_preHighWater.load();
    while ((fill > highWater) && !m_preHighWater.compare_exchange_weak(highWater, fill)) {
    }
    if (written < inputSamples) {
        m_preDroppedSamples.fetch_add(inputSamples - written);
    }
    m_preWaitCondition.notify_one();
}

void BFMSharedVFOBranch::processDirect()
{
    while (!m_preStop.load()) {
        {
            std::unique_lock<std::mutex> lock(m_preWaitMutex);
            m_preWaitCondition.wait(lock, [this]() {
                return m_preStop.load() || (m_rawFifo.fill() > 0);
            });
        }

        while (m_rawFifo.fill() > 0) {
            SampleVector::iterator part1begin;
            SampleVector::iterator part1end;
            SampleVector::iterator part2begin;
            SampleVector::iterator part2end;
            const unsigned int count = m_rawFifo.readBegin(
                std::min(m_rawFifo.fill(), 1U << 20),
                &part1begin,
                &part1end,
                &part2begin,
                &part2end);

            QElapsedTimer processingTimer;
            processingTimer.start();
            m_directBuffer.resize(static_cast<int>(count / m_decimation) + 2);
            SampleVector::iterator output = m_directBuffer.begin();

            auto processPart = [this, &output](SampleVector::iterator begin, SampleVector::iterator end) {
                if (begin == end) {
                    return;
                }

                const unsigned int inputSamples = static_cast<unsigned int>(end - begin);
                const qint32* raw = reinterpret_cast<const qint32*>(&*begin);
                const qint32 rawLength = static_cast<qint32>(inputSamples * 2);

                if (m_decimation <= 64) {
                    switch (m_decimation) {
                    case 2: m_decimators1.decimate2_cen(&output, raw, rawLength); break;
                    case 4: m_decimators1.decimate4_cen(&output, raw, rawLength); break;
                    case 8: m_decimators1.decimate8_cen(&output, raw, rawLength); break;
                    case 16: m_decimators1.decimate16_cen(&output, raw, rawLength); break;
                    case 32: m_decimators1.decimate32_cen(&output, raw, rawLength); break;
                    case 64: m_decimators1.decimate64_cen(&output, raw, rawLength); break;
                    default: break;
                    }
                } else {
                    m_directIntermediate.resize(static_cast<int>(inputSamples / 64) + 2);
                    SampleVector::iterator firstOutput = m_directIntermediate.begin();
                    m_decimators1.decimate64_cen(&firstOutput, raw, rawLength);
                    const unsigned int firstCount = static_cast<unsigned int>(firstOutput - m_directIntermediate.begin());
                    const qint32* firstRaw = reinterpret_cast<const qint32*>(m_directIntermediate.data());
                    m_decimators2.decimate2_cen(&output, firstRaw, static_cast<qint32>(firstCount * 2));
                }
            };

            processPart(part1begin, part1end);
            processPart(part2begin, part2end);
            m_rawFifo.readCommit(count);

            const unsigned int outputSamples = static_cast<unsigned int>(output - m_directBuffer.begin());
            m_directInputSamples += count;
            m_directOutputSamples += outputSamples;
            m_directProcessingNsecs += processingTimer.nsecsElapsed();

            if (outputSamples > 0) {
                m_worker->feed(m_directBuffer.cbegin(), m_directBuffer.cbegin() + outputSamples);
            }

            if (m_directReportTimer.isValid() && (m_directReportTimer.elapsed() >= 2000)) {
        const double seconds = static_cast<double>(m_directReportTimer.elapsed()) / 1000.0;
        qDebug() << "BFMSharedVFOBranch::directDecimation:"
                 << "inputRate:" << (static_cast<double>(m_directInputSamples) / seconds)
                 << "outputRate:" << (static_cast<double>(m_directOutputSamples) / seconds)
                 << "decimation:" << m_decimation
                 << "processingTimeMs:" << (static_cast<double>(m_directProcessingNsecs) / 1000000.0)
                 << "processingUtilizationPercent:"
                 << (100.0 * static_cast<double>(m_directProcessingNsecs) /
                     (static_cast<double>(m_directReportTimer.elapsed()) * 1000000.0));
                qDebug() << "BFMSharedVFOBranch::preInput:"
                         << "fifoFill:" << m_rawFifo.fill()
                         << "fifoSize:" << m_rawFifo.size()
                         << "fifoHighWater:" << m_preHighWater.load()
                         << "droppedSamples:" << m_preDroppedSamples.load();
        m_directInputSamples = 0;
        m_directOutputSamples = 0;
        m_directProcessingNsecs = 0;
        m_directReportTimer.restart();
    } else if (!m_directReportTimer.isValid()) {
        m_directReportTimer.start();
            }

            if (m_preStop.load()) {
                break;
            }
        }
    }
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
