///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020 Edouard Griffiths, F4EXB <f4exb06@gmail.com>               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include "decimatorc.h"
#include "dsp/nco.h"

#include <algorithm>

DecimatorC::DecimatorC() :
    m_log2Decim(0),
    m_decim(1)
{}

void DecimatorC::setLog2Decim(unsigned int log2Decim)
{
    m_log2Decim = log2Decim;
    m_decim = 1 << log2Decim;
}

bool DecimatorC::decimate(Complex c, Complex& cd)
{
    if (m_log2Decim == 0) { // no decimation hence no translation possible
        return true;
    }

    if (m_log2Decim == 1) {
        return decimate2(c, cd);
    } else if (m_log2Decim == 2) {
        return decimate4(c, cd);
    } else if (m_log2Decim == 3) {
        return decimate8(c, cd);
    } else if (m_log2Decim == 4) {
        return decimate16(c, cd);
    } else if (m_log2Decim == 5) {
        return decimate32(c, cd);
    } else if (m_log2Decim == 6) {
        return decimate64(c, cd);
    } else {
        return true; // no decimation
    }
}

unsigned int DecimatorC::decimateBlock(
        const Complex* input,
        unsigned int inputCount,
        Complex* output)
{
    if (!input || !output || inputCount == 0) {
        return 0;
    }

    if (m_log2Decim == 0) {
        std::copy_n(input, inputCount, output);
        return inputCount;
    }

    auto process = [this, input, inputCount, output](auto decimateFunction) {
        unsigned int outputCount = 0;

        for (unsigned int i = 0; i < inputCount; ++i) {
            if ((this->*decimateFunction)(input[i], output[outputCount])) {
                ++outputCount;
            }
        }

        return outputCount;
    };

    switch (m_log2Decim)
    {
    case 1:
        return process(&DecimatorC::decimate2);
    case 2:
        return process(&DecimatorC::decimate4);
    case 3:
        return process(&DecimatorC::decimate8);
    case 4:
        return process(&DecimatorC::decimate16);
    case 5:
        return process(&DecimatorC::decimate32);
    case 6:
        return process(&DecimatorC::decimate64);
    default:
        return 0;
    }
}

unsigned int DecimatorC::decimateBlockNCO(
        const Sample* input,
        unsigned int inputCount,
        NCO& nco,
        Complex* output)
{
    if (!input || !output || inputCount == 0) {
        return 0;
    }

    auto process = [this, input, inputCount, &nco, output](auto decimateFunction) {
        unsigned int outputCount = 0;

        for (unsigned int i = 0; i < inputCount; ++i) {
            const Complex normalizedInput(
                input[i].real() / SDR_RX_SCALEF,
                input[i].imag() / SDR_RX_SCALEF);
            const Complex translated = normalizedInput * nco.nextIQ();

            if ((this->*decimateFunction)(translated, output[outputCount])) {
                ++outputCount;
            }
        }

        return outputCount;
    };

    switch (m_log2Decim)
    {
    case 0:
        for (unsigned int i = 0; i < inputCount; ++i) {
            const Complex normalizedInput(
                input[i].real() / SDR_RX_SCALEF,
                input[i].imag() / SDR_RX_SCALEF);
            output[i] = normalizedInput * nco.nextIQ();
        }
        return inputCount;
    case 1:
        return process(&DecimatorC::decimate2);
    case 2:
        return process(&DecimatorC::decimate4);
    case 3:
        return process(&DecimatorC::decimate8);
    case 4:
        return process(&DecimatorC::decimate16);
    case 5:
        return process(&DecimatorC::decimate32);
    case 6:
        return process(&DecimatorC::decimate64);
    default:
        return 0;
    }
}

bool DecimatorC::decimate2(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        cd.real(x);
        cd.imag(y);
    }

    return done2;
}

bool DecimatorC::decimate4(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        bool done4 = m_decimator4.workDecimateCenter(&x, &y);

        if (done4)
        {
            cd.real(x);
            cd.imag(y);
            return true;
        }
    }

    return false;
}

bool DecimatorC::decimate8(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        bool done4 = m_decimator4.workDecimateCenter(&x, &y);

        if (done4)
        {
            bool done8 = m_decimator8.workDecimateCenter(&x, &y);

            if (done8)
            {
                cd.real(x);
                cd.imag(y);
                return true;
            }
        }
    }

    return false;
}

bool DecimatorC::decimate16(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        bool done4 = m_decimator4.workDecimateCenter(&x, &y);

        if (done4)
        {
            bool done8 = m_decimator8.workDecimateCenter(&x, &y);

            if (done8)
            {
                bool done16 = m_decimator16.workDecimateCenter(&x, &y);

                if (done16)
                {
                    cd.real(x);
                    cd.imag(y);
                    return true;
                }
            }
        }
    }

    return false;
}


bool DecimatorC::decimate32(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        bool done4 = m_decimator4.workDecimateCenter(&x, &y);

        if (done4)
        {
            bool done8 = m_decimator8.workDecimateCenter(&x, &y);

            if (done8)
            {
                bool done16 = m_decimator16.workDecimateCenter(&x, &y);

                if (done16)
                {
                    bool done32 = m_decimator32.workDecimateCenter(&x, &y);

                    if (done32)
                    {
                        cd.real(x);
                        cd.imag(y);
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

bool DecimatorC::decimate64(Complex c, Complex& cd)
{
    float x = c.real();
    float y = c.imag();
    bool done2 = m_decimator2.workDecimateCenter(&x, &y);

    if (done2)
    {
        bool done4 = m_decimator4.workDecimateCenter(&x, &y);

        if (done4)
        {
            bool done8 = m_decimator8.workDecimateCenter(&x, &y);

            if (done8)
            {
                bool done16 = m_decimator16.workDecimateCenter(&x, &y);

                if (done16)
                {
                    bool done32 = m_decimator32.workDecimateCenter(&x, &y);

                    if (done32)
                    {
                        bool done64 = m_decimator64.workDecimateCenter(&x, &y);

                        if (done64)
                        {
                            cd.real(x);
                            cd.imag(y);
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}
