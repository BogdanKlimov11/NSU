from sleekxmpp import ClientXMPP
import requests
from lxml import etree
import time


#ADDR = 'monk@unboiled.info'
ADDR = 'networktest@xmpp.jp'


class Client(ClientXMPP):

    def __init__(self, jid, password):
        ClientXMPP.__init__(self, jid, password)

        self.add_event_handler("session_start", self.session_start)

    def session_start(self, event):
        self.send_presence()
        self.get_roster()


xmpp = Client('networksend@xmpp.jp', '12345678')
xmpp.connect()
xmpp.process()
s = requests.session()
last = None
while True:
    r = s.get('http://www.moex.com/iss/engines/currency/markets/selt/boards/CETS/securities/EUR_RUB__TOM.xml?iss.meta=off&iss.only=marketdata')
    tree = etree.XML(r.content)
    row = tree.xpath('/document/data[@id="marketdata"]/rows/row')[0]
    message_send = row.get("LAST")
    print(message_send)
    if (last == None):
        last = message_send
    else:
        if (last != message_send):
            last = message_send
            xmpp.send_message(mto=ADDR, mbody=('New EUR/RUB rate at MOEX: %s' % (last,)), mtype=('chat'))
    time.sleep(1)
