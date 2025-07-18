import sys
import signal
from gnuradio import gr
from gnuradio import blocks
from gnuradio import analog
from gnuradio import uhd
import time

import sys
import os
sys.path.insert(0, os.path.abspath('../../v2x_stack/gr-ieee802-11'))
# Import your IEEE 802.11 PHY layer from gr-ieee802-11
from ieee802_11 import ieee802_11_phy

class V2VTxRx(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "V2V TxRx")

        # UHD Source and Sink (PlutoSDR)
        self.tx_usrp = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(cpu_format="fc32", channels=[0]),
        )
        self.tx_usrp.set_center_freq(5.89e9)  # 5.9 GHz band for ITS
        self.tx_usrp.set_gain(40)
        self.tx_usrp.set_samp_rate(20e6)

        # Example data source: Random bytes or from a file
        self.src = blocks.vector_source_b([0x55, 0xAA]*1000, repeat=True)

        # Instantiate 802.11 PHY block from gr-ieee802-11 (assumed interface)
        self.ieee_phy = ieee802_11_phy()

        # Connect flowgraph: source -> 802.11 PHY -> USRP Sink
        self.connect(self.src, self.ieee_phy)
        self.connect(self.ieee_phy, self.tx_usrp)

def main():
    tb = V2VTxRx()

    def signal_handler(sig, frame):
        print("Stopping...")
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("Starting V2V TxRx flowgraph...")
    tb.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    tb.stop()
    tb.wait()

if __name__ == '__main__':
    main()
