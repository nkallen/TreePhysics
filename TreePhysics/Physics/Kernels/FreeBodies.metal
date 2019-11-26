#include <metal_stdlib>
#import "ShaderTypes.h"

using namespace metal;

struct arguments {
    command_buffer        cmd_buffer;
    device uint           *freeIndex;
    device ushort         *firstChildId;
    device uint           *parentId;
    device uchar          *childIndex;
    device uchar          *childCount;
};

kernel void
encodeFreeBodies(
                 device arguments      &args,
                 device uint           &freeBodyCount,
                 uint gid [[ thread_position_in_grid ]])
{
    compute_command cmd(args.cmd_buffer, gid);
    cmd.concurrent_dispatch_threads(uint3(freeBodyCount, 1, 1), uint3(1024, 1, 1));
    freeBodyCount = 0;
}


kernel void
freeBodies(
           device arguments      &args,
           uint gid [[ thread_position_in_grid ]])
{
    const uint id = args.freeIndex[gid];
    const uint parentId = args.parentId[id];
    const uchar childIndex = args.childIndex[id];

    const uint firstChildId = id - childIndex;
    const uint lastChildId = firstChildId + parentId;
    args.childIndex[lastChildId] = childIndex;
    args.childCount[parentId]--; // FIXME race

    args.parentId[id] = NO_PARENT;
    args.childIndex[id] = 0;
}
