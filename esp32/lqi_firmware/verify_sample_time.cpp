#include "hw/sample_time.h"
#include <cassert>
#include <cstdint>
#include <iostream>

int main()
{
    using motor_sample_time::after_start;
    const uint64_t starts[]={10000,19312500,0x7fffff00ULL,0xfffffff0ULL,0x100000010ULL,0x123456789abULL};
    const uint64_t offsets[]={0,1,200,1000,0x7fffffffULL,0x80000000ULL,0x800000c8ULL,0xffffffffULL,0x100000000ULL,0x300000001ULL,86400000000ULL};
    unsigned checks=0;
    for(uint64_t start:starts)
    {
        for(uint64_t elapsed:offsets)
        {
            for(uint64_t age:{0ULL,50ULL,200ULL,1050ULL})
            {
                const uint64_t sample=start+elapsed;
                assert(after_start(static_cast<uint32_t>(sample),sample+age,start));
                ++checks;
            }
        }
        for(uint64_t before:{1ULL,50ULL,500ULL})
        {
            assert(!after_start(static_cast<uint32_t>(start-before),start+200,start));
            ++checks;
        }
    }
    uint64_t random=914233;
    for(unsigned i=0;i<100000;++i)
    {
        random=random*6364136223846793005ULL+1;
        const uint64_t start=10000+(random>>16);
        const uint64_t elapsed=(random&0xffffffffULL)*7;
        const uint64_t sample=start+elapsed;
        assert(after_start(static_cast<uint32_t>(sample),sample+50,start));
        assert(!after_start(static_cast<uint32_t>(start-1),start+50,start));
        checks+=2;
    }
    // Reproduce 5kHz FOC accepting ADC samples until the signed 2^31 boundary,
    // with an ISR commit 50us later. ADC/heartbeat remain fresh in both cases.
    const uint64_t start=19312500,edge=0x80000000ULL;
    uint64_t old_command=start+edge-1200,new_command=old_command;
    uint64_t old_trip_age=0;
    for(uint64_t elapsed=edge-1000;elapsed<=edge+2000;elapsed+=200)
    {
        const uint64_t sample=start+elapsed;
        if(static_cast<int32_t>(static_cast<uint32_t>(sample)-static_cast<uint32_t>(start))>=0)old_command=sample;
        if(after_start(static_cast<uint32_t>(sample),sample+10,start))new_command=sample;
        if(!old_trip_age && sample+50-old_command>1000)old_trip_age=sample+50-old_command;
        assert(sample+50-new_command==50);
    }
    assert(old_trip_age==1050);
    std::cout<<"{\"checks\":"<<checks<<",\"old_boundary_trip_age_us\":"<<old_trip_age
             <<",\"new_latest_command_age_us\":50,\"result\":\"PASS\"}\n";
}
