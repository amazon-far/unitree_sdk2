import time
from typing import Any, Callable
import threading
from threading import Thread, Event

from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.internal import dds_c_t
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.core import DDSException, Listener
from cyclonedds.util import duration
from cyclonedds.internal import dds_c_t, InvalidSample

# for channel config
from .channel_config import ChannelConfigAutoDetermine, ChannelConfigHasInterface

# for singleton
from ..utils.singleton import Singleton
from ..utils.bqueue import BQueue
import logging
logger = logging.getLogger("dds")


"""
" class ChannelReader
"""

"""
" class Channel
"""
class Channel:
    
    """
    " internal class __Reader
    """
    class __Reader:
        def __init__(self):
            self.__reader = None
            self.__handler = None
            self.__queue = None
            self.__queueEnable = False
            self.__threadEvent = None
            self.__threadReader = None
            self.__listener = None
            
        
        def Init(self, participant: DomainParticipant, topic: Topic, qos: Qos = None, handler: Callable = None, queueLen: int = 0):
            if handler is None:
                # Non-handler path (polling mode) - use named args for consistency
                self.__reader = DataReader(participant, topic, qos=qos)
            else:
                # Handler path (callback mode)
                logger.debug("[DEBUG] __Reader.Init handler-path qos =", qos)
                
                self.__handler = handler
                if queueLen > 0:
                    self.__queueEnable = True
                    self.__queue = BQueue(queueLen)
                    self.__threadEvent = Event()
                    self.__threadReader = Thread(target=self.__ChannelReaderThreadFunc, name="ch_reader", daemon=True)
                    self.__threadReader.start()
                
                # Create listener and reader with named arguments to prevent signature mismatch
                self.__listener = Listener(on_data_available=self.__OnDataAvailable)
                self.__reader = DataReader(participant, topic, qos=qos, listener=self.__listener)
                
                logger.debug("[DEBUG] created reader =", self.__reader)

        def Read(self, timeout: float = None):
            sample = None
            try:
                if timeout is None:
                    sample = self.__reader.take_one()
                else:
                    # Fix timeout handling - convert to nanoseconds properly
                    ns = int(timeout * 1e9)
                    sample = self.__reader.take_one(timeout=duration(nanoseconds=ns))
            except StopIteration as e:
                logger.debug("[Reader] catch StopIteration msg:", e)
            except DDSException as e:
                logger.debug("[Reader] catch DDSException msg:", e.msg)
            except TimeoutError as e:
                logger.debug("[Reader] take sample timeout")
            except Exception as e:
                import traceback
                logger.debug("ERROR in Read():", repr(e))
                traceback.print_exc()

            return sample

        def Close(self):
            if self.__reader is not None:
                del self.__reader

            self.__listener = None

            if self.__queueEnable:
                self.__threadEvent.set()
                self.__queue.Interrupt()
                self.__queue.Clear()
                self.__threadReader.join()

        def __OnDataAvailable(self, reader: DataReader):
            samples = []
            try:
                samples = reader.take(1)
            except DDSException as e:
                logger.debug("[Reader] catch DDSException error. msg:", e.msg)
                return
            except TimeoutError as e:
                logger.debug("[Reader] take sample timeout")
                return
            except Exception as e:
                import traceback
                logger.debug("ERROR in __OnDataAvailable():", repr(e))
                traceback.print_exc()
                return

            if samples is None:
                return

            # check invalid sample        
            sample = samples[0]
            if isinstance(sample, InvalidSample):
                return

            # do sample
            if self.__queueEnable:
                self.__queue.Put(sample)
            else:
                self.__handler(sample)

        def __ChannelReaderThreadFunc(self):
            while not self.__threadEvent.is_set():
                sample = self.__queue.Get()
                if sample is not None:
                    self.__handler(sample)

    """
    " internal class __Writer
    """
    class __Writer:
        def __init__(self):
            self.__writer = None
            self.__publication_matched_count = 0
            self.__listener = None
        
        def Init(self, participant: DomainParticipant, topic: Topic, qos: Qos = None):
            self.__listener = Listener(on_publication_matched=self.__OnPublicationMatched)
            self.__writer = DataWriter(participant, topic, qos, self.__listener)
            time.sleep(0.2)

        def Write(self, sample: Any, timeout: float = None):
            waitsec = 0.0 if timeout is None else timeout

            # check publication_matched_count
            while waitsec > 0.0 and self.__publication_matched_count == 0:
                time.sleep(0.1)
                waitsec = waitsec - 0.1
            #   logger.debug(time.time())

            # check waitsec
            if timeout is not None and waitsec <= 0.0:
                return False

            try:
                self.__writer.write(sample)
            except DDSException as e:
                logger.debug("[Writer] catch DDSException error. msg:", e.msg)
                return False
            except Exception as e:
                logger.debug("[Writer] write sample error. msg:", e.args())
                return False

            return True
        
        def Close(self):
            if self.__writer is not None:
                del self.__writer
            self.__listener = None
        
        def __OnPublicationMatched(self, writer: DataWriter, status: dds_c_t.publication_matched_status):
            self.__publication_matched_count = status.current_count


    # channel __init__
    def __init__(self, participant: DomainParticipant, name: str, type: Any, qos: Qos = None):
        self.__reader = self.__Reader()
        self.__writer = self.__Writer()
        self.__participant = participant
        self.__topic = Topic(self.__participant, name, type, qos)

    def SetWriter(self, qos: Qos = None):
        self.__writer.Init(self.__participant, self.__topic, qos)

    def SetReader(self, qos: Qos = None, handler: Callable = None, queueLen: int = 0):
        # Force Reliable QoS for callback-based subscribers if no QoS is provided
        if handler is not None and qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(max_blocking_time=10000),
                Policy.History.KeepLast(1),
            )
            logger.debug("[DEBUG] SetReader: forcing Reliable QoS for handler-based reader")
        self.__reader.Init(self.__participant, self.__topic, qos, handler, queueLen)
        
    def Write(self, sample: Any, timeout: float = None):
        return self.__writer.Write(sample, timeout)

    def Read(self, timeout: float = None):
        return self.__reader.Read(timeout)

    def CloseReader(self):
        self.__reader.Close()

    def CloseWriter(self):
        self.__writer.Close()


"""
" class ChannelFactory
"""
class ChannelFactory(Singleton):
    __domain = None
    __participant = None
    __qos = None

    __initialized = False
    __init_lock = threading.Lock()

    def __init__(self):
        super().__init__()

    def Init(self, id: int, networkInterface: str = None, qos: Qos = None, config: str = ""):
        if self.__class__.__initialized:
            return True
        
        with self.__class__.__init_lock:
            if self.__class__.__initialized:
                return True
                        
            if not config:
                # choose config
                if networkInterface is None:
                    config = ChannelConfigAutoDetermine
                else:
                    config = ChannelConfigHasInterface.replace('$__IF_NAME__$', networkInterface)
            
            logger.debug(f"using DDS config {config}")

            try:
                self.__class__.__domain = Domain(id, config)
            except DDSException as e:
                logger.debug("[ChannelFactory] create domain error. msg:", e.msg)
                return False
            except:
                logger.debug("[ChannelFactory] create domain error.")
                return False

            try:
                self.__class__.__participant = DomainParticipant(id)
            except DDSException as e:
                logger.debug("[ChannelFactory] create domain participant error. msg:", e.msg)
                return False
            except:
                logger.debug("[ChannelFactory] create domain participant error")
                return False

            self.__class__.__qos = qos
            self.__class__.__initialized = True
            return True

    def CreateChannel(self, name: str, type: Any):
        return Channel(self.__class__.__participant, name, type, self.__class__.__qos)

    def CreateSendChannel(self, name: str, type: Any):
        channel = self.CreateChannel(name, type)
        channel.SetWriter(None)
        return channel

    def CreateRecvChannel(self, name: str, type: Any, handler: Callable = None, queueLen: int = 0):
        channel = self.CreateChannel(name, type)
        channel.SetReader(None, handler, queueLen)
        return channel


"""
" class ChannelPublisher
"""
class ChannelPublisher:
    def __init__(self, name: str, type: Any):
        factory = ChannelFactory()
        self.__channel = factory.CreateChannel(name, type)
        self.__inited = False

    def Init(self):
        if not self.__inited:
            self.__channel.SetWriter(None)
            self.__inited = True

    def Close(self):
        self.__channel.CloseWriter()
        self.__inited = False

    def Write(self, sample: Any, timeout: float = None):
        return self.__channel.Write(sample, timeout)

"""
" class ChannelSubscriber
"""
class ChannelSubscriber:
    def __init__(self, name: str, type: Any):
        factory = ChannelFactory()
        self.__channel = factory.CreateChannel(name, type)
        self.__inited = False

    def Init(self, handler: Callable = None, queueLen: int = 0, qos: Qos = None):
        if not self.__inited:
            if qos is None:
                qos = Qos(
                    Policy.Reliability.Reliable(max_blocking_time=10000),
                    Policy.History.KeepLast(1),
                )
            self.__channel.SetReader(qos, handler, queueLen)
            self.__inited = True

    def Close(self):
        self.__channel.CloseReader()
        self.__inited = False

    def Read(self, timeout: int = None):
        return self.__channel.Read(timeout)

"""
" function ChannelFactoryInitialize. used to intialize channel everenment.
"""
def ChannelFactoryInitialize(id: int = 0, networkInterface: str = None, config: str = ""):
    factory = ChannelFactory()
    if not factory.Init(id=id, networkInterface=networkInterface, config=config):
        raise Exception("channel factory init error.")
