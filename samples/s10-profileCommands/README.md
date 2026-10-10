# Profiling GPU commands

`gprtBeginProfile` returns a `GPRTEvent` containing a timeline semaphore and its
submitted value. GPRT commands submitted during the region wait for this start
event. Independent queues can run concurrently.

Capture a sort's completion point and pass it to `gprtEndProfile`:

```cpp
auto start = gprtBeginProfile(context);
gprtBufferSort(context, keys, scratch);
auto sorted = gprtGetQueueEvent(context, GPRT_QUEUE_COMPUTE);
float milliseconds = gprtEndProfile(context, 1, &sorted);
```

Use `gprtQueueWait` to connect graph nodes. For example, make the graphics queue
wait for a sort before launching a raygen that reads the sorted keys:

```cpp
gprtBeginProfile(context);
gprtBufferSort(context, keys, scratch);
auto sorted = gprtGetQueueEvent(context, GPRT_QUEUE_COMPUTE);
gprtQueueWait(context, GPRT_QUEUE_GRAPHICS, 1, &sorted);
gprtRayGenLaunch1D(context, raygen, count);
auto rendered = gprtGetQueueEvent(context, GPRT_QUEUE_GRAPHICS);
GPRTEvent leaves[] = {sorted, rendered};
float milliseconds = gprtEndProfile(context, 2, leaves);
```

The runnable sample demonstrates a mixed graph and checks the GPU results.
A timeline point includes prior
work on its queue. Capture each leaf immediately after its work, and include every
independent leaf in the end-event array. Events must come from the same context;
callers must not destroy or signal their semaphores.

The optional event array on `gprtBeginProfile` excludes the wait for specified
producers from the measured interval. For example, pass an upload event to start
timing after the upload. With no event array, `gprtEndProfile(context)` includes
all GPRT submissions in the region and preserves existing source usage.

Profiling covers submissions through the graphics, compute, and transfer queues,
including sorts with or without payloads, compute launches, raygen launches,
acceleration builds/updates/compaction, buffer and texture copies, uploads,
readbacks, texture operations, GUI rasterization, and presentation copies.
Prefix sum, partition, and select currently use an unimplemented stub. There are
no public unique or histogram commands. Implementations that use GPRT's submission
helpers inherit this profiling support.

Results measure elapsed time between two graphics-queue GPU timestamps. This
includes host submission gaps, queue waits, and any earlier graphics work queued
before the end marker. It does not sum individual kernel execution times or
measure display scanout. Existing blocking APIs retain their waits. Warm up
pipelines and allocate resources before the region when measuring steady work.

Only one region may be active per context. Empty regions return zero. Invalid
calls use `LOG_ERROR`; if the handler returns, begin returns an empty event and
end returns `-1`. A completion timeout keeps the region active; retry end before
beginning another region. Rebuild clients when adopting the new API signatures.
