""" This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

import time
import ntcore
from typing import cast

def consume(e:ntcore.Event):
    if isinstance(e, ntcore.TimeSyncEventData):
        tsed = cast(ntcore.TimeSyncEventData, e.data)
        print(f"Time Sync Event: {tsed.serverTimeOffset} {tsed.rtt2} {tsed.valid}")

def main() -> None:
    print("*** main")

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("sync_client")
    inst.setServer("10.1.0.2")

    servernow_sub = inst.getIntegerTopic("servernow").subscribe(0)
 
    nowpi_pub = inst.getIntegerTopic("/nowpi").publish()
    servernowtime_pub = inst.getIntegerTopic("/servernowtime").publish()
    nowdiff_pub = inst.getIntegerTopic("/nowdiff").publish()
    offset_pub = inst.getIntegerTopic("/offset").publish()

    listener = ntcore.NetworkTableListener.createTimeSyncListener(
            inst, True, consume);

    while True:
        # the value and the servertime are the same
        servernow = servernow_sub.getAtomic()

        now = ntcore._now()
        nowpi_pub.set(now)

        # local time of the message
        servernowtime_pub.set(servernow.time)

        # difference between *local* time and the pi "now" measurement
        nowdiff_pub.set(servernow.time - now)

        # offset is never observed to change  
        offset = ntcore.NetworkTableInstance.getDefault().getServerTimeOffset()
        if offset is not None:
            offset_pub.set(offset)

        # avoid spinning too fast
        time.sleep(0.02)
