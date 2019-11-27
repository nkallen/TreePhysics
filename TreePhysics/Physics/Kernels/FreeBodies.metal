#include <metal_stdlib>
#import "ShaderTypes.h"

using namespace metal;

struct arguments {
    command_buffer        cmd_buffer;
    device uint           *toBeFreedIndex;
    device uint           *freeBodyIndex;
    device atomic_uint    &freeBodyCount;
    device ushort         *firstChildId;
    device uint           *parentId;
    device uchar          *childIndex;
    device uchar          *childCount;
};

kernel void
encodeFreeBodies(
                 device arguments      &args,
                 device uint           &toBeFreedCount,
                 uint gid [[ thread_position_in_grid ]])
{
    compute_command cmd(args.cmd_buffer, gid);
    cmd.concurrent_dispatch_threads(uint3(toBeFreedCount, 1, 1), uint3(1024, 1, 1));
    toBeFreedCount = 0;
}


kernel void
freeBodies(
           device arguments      &args,
           uint gid [[ thread_position_in_grid ]])
{
    const uint id = args.toBeFreedIndex[gid];
    const uint parentId = args.parentId[id];
    const uchar childIndex = args.childIndex[id];

    uint freeBodyId = atomic_fetch_add_explicit(&args.freeBodyCount, 1, memory_order_relaxed);
    args.freeBodyIndex[freeBodyId] = id;

    const uint firstChildId = id - childIndex;
    const uint lastChildId = firstChildId + parentId;
    args.childIndex[lastChildId] = childIndex;
    args.childCount[parentId]--; // FIXME race

    args.parentId[id] = NO_PARENT;
    args.childIndex[id] = 0;
}
