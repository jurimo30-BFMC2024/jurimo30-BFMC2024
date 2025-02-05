# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

from collections import defaultdict
from threading import Lock
from concurrent.futures import ThreadPoolExecutor
from src.templates.threadwithstop import ThreadWithStop
from src.gateway.PriorityQueueHandler import PriorityQueueHandler

class threadGateway(ThreadWithStop):
    """Thread handling inter-process messages with priority queuing."""

    def __init__(self, queueList, logger, debugging):
        super().__init__()
        self.logger = logger
        self.debugging = debugging
        self.sendingList = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
        self.handler = PriorityQueueHandler(queueList, logger, debugging)
        self.messageApproved = set()  # Use a set for O(1) lookups
        self.executor = ThreadPoolExecutor(max_workers=4)  # Reusable thread pool
        self.pipe_locks = defaultdict(Lock)  # Locks for each pipe

    def subscribe(self, message):
        """Add a receiver to the approved list."""
        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]
        Pipe = message["To"]["pipe"]

        self.sendingList[Owner][Id][To] = Pipe
        self.messageApproved.add((Owner, Id))

        if self.debugging:
            self.logger.warning(f"Subscribed: {self.sendingList}")

    def unsubscribe(self, message):
        """Remove a receiver from the approved list."""
        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]

        del self.sendingList[Owner][Id][To]
        self.messageApproved.remove((Owner, Id))

        if self.debugging:
            self.logger.warning(f"Unsubscribed: {self.sendingList}")

    def send(self, message):
        """Send a message to all subscribed receivers in parallel."""
        Owner = message["Owner"]
        Id = message["msgID"]
        Type = message["msgType"]
        Value = message["msgValue"]

        if (Owner, Id) in self.messageApproved:
            msg = {"Type": Type, "value": Value, "id": Id, "Owner": Owner}
            
            # Submit send tasks to the thread pool
            futures = []
            for receiver, pipe in self.sendingList[Owner][Id].items():
                futures.append(
                    self.executor.submit(self._send_to_pipe, pipe, msg)
                )
            
            if self.debugging:
                self.logger.warning(f"Sent: {msg}")

    def _send_to_pipe(self, pipe, message):
        """Thread-safe method to send a message through a pipe."""
        with self.pipe_locks[pipe]:  # Lock the pipe for this thread
            pipe.send(message)

    def run(self):
        """Process messages in priority order."""
        while self._running:
            priority, message = self.handler.get()
            if priority == "Config":
                action = message["Subscribe/Unsubscribe"].lower()
                if action == "subscribe":
                    self.subscribe(message)
                else:
                    self.unsubscribe(message)
            else:
                self.send(message)