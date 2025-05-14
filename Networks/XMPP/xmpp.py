from sleekxmpp import ClientXMPP
import requests
from xml.etree import ElementTree
import time

ADDR = 'mynewlogin@xmpp.jp'

class Client(ClientXMPP):
    def __init__(self, jid, password):
        ClientXMPP.__init__(self, jid, password)

        self.add_event_handler("session_start", self.session_start)
        self.add_event_handler("message", self.message)

    def session_start(self, event):
        self.send_presence()
        self.get_roster()

    def message(self, msg):
        if msg['type'] in ('chat', 'normal'):
            msg.reply("U r so lonely that u r writing to bot?").send()

xmpp = Client('mynewbot@xmpp.jp', '12345678')
xmpp.connect()
xmpp.process()
s = requests.session()
last = None

while True:
    r = s.get('http://www.moex.com/iss/engines/currency/markets/selt/boards/CETS/securities/EUR_RUB__TOM.xml?iss.meta=off&iss.only=marketdata')
    now = ElementTree.fromstring(r.text)[0][0][0].attrib['LAST']
    print("now: " + now)
    
    if (last == None):
        last = now
    else:
        if (last != now):
            last = now
            xmpp.send_message(mto=ADDR, mbody=('New EUR/RUB rate at MOEX: %s' % (last,)), mtype=('chat'))
    
    time.sleep(1)
