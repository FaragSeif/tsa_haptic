

import lcm
import select as select

# TODO: MOVE EVERYTHING TO CLASS

# address =
UDP_MULTICAST = "udpm://239.255.76.67:7667?ttl=1"
UDP_MULTICAST = ""
# example =
# channels_dict = {'controller': controller_states_t,
#                  'commands': commands_t,
#                  'device': device_states_t}

# TODO:
# HANDLE MULTIPLE SUBCRIPTIONS !


class LCM_Channel(object):
    def __init__(self, lcm, name, struct):
        self.lcm = lcm
        self.lc = lcm.lc

        self.struct = struct
        self.slots = self.struct.__slots__
        self.name = name
        self.update = self.__update_channel

        self.subscription = None
        self.message = struct()
        self.__msg2attr()

    def __msg2attr(self):
        for slot in self.slots:
            vars(self)[slot] = getattr(self.message, slot)

    def __attr2msg(self):
        for slot in self.slots:
            setattr(self.message, slot, getattr(self, slot)) 

    def __update_channel(self, channel, data):
        channel_data = self.struct.decode(data)
        for slot in self.slots:
            vars(self)[slot] = getattr(channel_data, slot)
        # self.__msg2attr()

    def subscribe(self):
        if self.subscription is not None:
            print(f'[LCM] Already subscribed to "{self.name}" channel')
        else:
            self.subscription = self.lc.subscribe(self.name, self.__update_channel)
            # self.lcm.subscriptions[self.name] = self.name
            print(f'[LCM] Subscribed to "{self.name}" channel')

    def publish(self, update_message=True):
        if update_message:
            self.__attr2msg()
        self.lc.publish(self.name, self.message.encode())

    def unsubscribe(self):
        if self.subscription is not None:
            self.lc.unsubscribe(self.subscription)
            self.subscription = None
            print(f'[LCM] Unsubscribed from "{self.name}" channel')

        else:
            print(f'[LCM] There is no subscription to "{self.name}" channel')


class LCM_Handler(object):
    def __init__(self,
                 address=UDP_MULTICAST):

        self.lc = lcm.LCM(address)
        self.subscriptions = {}

    def handle(self, blocking=False):
        handled = False
        if not blocking:
            rfds, _, _ = select.select([self.lc.fileno()], [], [], 0.)
            if rfds:
                self.lc.handle()
                self.handled = True
        else:
            self.lc.handle()
            self.handled = True
        return handled

    def unsubscribe(self):
        pass

    def subscribe(self):
        pass


# TODO: LCM INTERFACE IS HANDLER TO CREATE BOTH LCM NETWORK AND LCM CHANNELS

# class LCM_Interface(object):
#     def __init__(self,
#                  channels_mapping=None,
#                  address=UDP_MULTICAST):

#         # if channels is None:
#         #     print('Provide the labels for channels!')
#         self.channel_names = channels_mapping.keys()

#         self.lc = lcm.LCM(address)

#         for channel in self.channel_names:
#             vars(self)[channel] = LCM_Channel(name=channel,
#                                               protocol=channels_mapping[channel])
#             vars(self)[channel].subscription = None
#             # do not
#             vars(self)[channel].subscribe = lambda: self.subscribe(
#                 channel=vars(self)[channel])
#             vars(self)[channel].unsubscribe = self.unsubscribe(channel)
#             vars(self)[channel].message = channels_mapping[channel]()

#         # # self.channels =
#         # for channel in channels:
#         #     vars(self)[channel] = SimpleNamespace()
#         #     vars(self)[channel].message = None
#         #     vars(self)[channel].subscribe = None
#         #     vars(self)[channel].unsubscribe = None
#         #     vars(self)[channel].update = None

#     def subscribe(self, channel):
#         channel.subscription = self.lc.subscribe(channel.name,
#                                                  channel.update)
#         print(f'[LCM] Subscribed to "{channel.name}" channel')

#     def publish(self, channel):
#         self.lc.publish(channel, channel.message.encode())

#     def unsubscribe(self, channel):
#         self.lc.unsubscribe(channel.subscription)
#         self.channel.subscription = None

#     def handle(self, blocking=False):
#         handled = False
#         if not blocking:
#             rfds, _, _ = select.select([self.lc.fileno()], [], [], 0.)
#             if rfds:
#                 self.lc.handle()
#                 self.handled = True
#         else:
#             self.lc.handle()
#             self.handled = True
#         return handled
