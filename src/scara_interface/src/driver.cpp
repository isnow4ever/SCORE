/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yanyu Su
* State Key Laboratory of Robotics and System, Harbin Institute of Technology
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include <xcdev/PMAC/protocol.h>

int main()
{
    xc::dev::pmac::Connector connector;
    std::cout << "connecting" << std::endl;

    connector.connect();
                              // ip
    connector.flush();        // flush
    connector.sendLine("?");  // send line "?"
    connector.readReady();    // read ready

    while(true)
    {
        char str[1024];
        std::cin.getline(str,1024);
        xc::dev::pmac::CommArrayPtr ret = connector.getResponse(str);
        for (size_t i = 0; i < ret->size(); ++i)
        {
            if (isprint(ret->at(i)))
                std::cout << ret->at(i);
            else
                std::cout << ".";
        }
        std::cout << std::endl;
    }
    return 1;
}
