# UART Image Processing SoC

**Interactive Project Demo:**
https://assaf-afriat.github.io/uart-image-processing-soc/docs/project_demo.html

A multi-clock UART-based System-on-Chip designed in SystemVerilog for real-time 256x256 RGB image transfer between a host PC and an FPGA. Implements a full protocol stack (PHY, MAC, Classifier, CDC) with DMA-style burst engine and register file infrastructure.

**Target**: Digilent Nexys A7-100T (Artix-7 XC7A100T)  
**Baud Rate**: 5.5 Mbaud  
**HDL**: SystemVerilog  
**Tools**: Vivado 2024.1, Python 3.x

---

## Architecture

```
  Host PC                         FPGA (Nexys A7-100T)
 ─────────                    ──────────────────────────────────────────────
                              176 MHz Domain            100 MHz Domain
  Python   ──── UART ────►  RX PHY ► RX MAC ► Classifier ► CDC ► Sequencers ► SRAM (R/G/B)
  Scripts  ◄─── UART ────  TX PHY ◄ TX MAC ◄ Composer   ◄ CDC ◄ Sequencers ◄ SRAM (R/G/B)
                                                           │
                                                           ├──► RGF Manager ► Register Files
                                                           └──► rx_msg_handler (unified handshake)
```

### Data Flow

1. **PC to FPGA (Image Write)**: Python script sends a `START_BURST_WR` announcement followed by 16,384 pixel messages. Each message carries 4 interleaved RGB pixels. The RX chain deserializes, validates, classifies, crosses clock domains, and the burst sequencer writes to three color-separated SRAMs.

2. **FPGA to PC (Image Read)**: Python script sends a `START_BURST_RD` request. The TX burst sequencer reads all 16K SRAM rows, packs them into pixel messages through async FIFOs, and the TX chain serializes them back to the host.

3. **Register Access**: The host can write/read any register file (LED, SYS, CNT, PWM, IMG) via structured 16-byte or 6-byte messages. Read responses are composed and transmitted back.

---

## Key Features

- **Layered Protocol Stack** -- PHY (bit-level, 32x oversampling), MAC (byte assembly & frame validation), Classifier (message parsing & field extraction)
- **Clock Domain Crossing** -- 2-FF / 3-FF synchronizers with edge detection, registered glitch guards, and aligned data sampling between 176 MHz and 100 MHz domains
- **DMA-Style Burst Engine** -- Dedicated RX and TX sequencers for streaming 256x256 images (65,536 pixels) without CPU intervention
- **Async FIFO** -- Gray-code pointer based FIFO for TX path CDC (SRAM @ 100 MHz to TX MAC @ 176 MHz)
- **6 Register Files** -- LED, SYS, CNT, PWM, IMG, and TX FIFO control registers with read/write access from host
- **Frame-Safe MAC** -- Handles pixel data matching delimiter characters (0x7D = `}`) without premature message termination
- **Glitch-Free Infrastructure** -- Clock multiplexer and async reset synchronizer

---

## Clock Domains

| Domain   | Frequency | Modules |
|----------|----------:|---------|
| Fast CLK | 176 MHz   | RX PHY, RX MAC, Classifier, CDC (RX side), TX PHY, TX MAC, Composer |
| Sys CLK  | 100 MHz   | Sequencers, RGF Manager, SRAM, rx_msg_handler, CDC (SYS side) |

---

## Message Protocol

All messages use ASCII-framed delimiters: `{`, `,`, `}`

| Type | Bytes | Format |
|------|------:|--------|
| RGF Write | 16 | `{W<addr><off_h><off_l>,V<dh1><dh0><pad>,V<dl1><dl0><pad>}` |
| RGF Read Request | 6 | `{R<addr><off_h><off_l>}` |
| RGF Read Response | 16 | `{R<addr><off_h><off_l>,V<pad><d31:24><d23:16>,V<pad><d15:8><d7:0>}` |
| Start Burst Write | 16 | `{I<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}` |
| Start Burst Read | 16 | `{R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}` |
| Burst Pixel | 16 | `{R0,G0,B0,R1,G1,B1,R2,G2,B2,R3,G3,B3}` |

Each burst pixel message carries **4 pixels** in interleaved RGB order. A 256x256 image requires 16,384 messages.

---

## Project Structure

```
├── rtl/                             # Synthesizable RTL (27 modules)
│   ├── uart_top.sv                  #   Top-level integration
│   ├── uart_types.sv                #   Shared type definitions
│   ├── Uart_Rx/                     #   RX: PHY, MAC, Classifier, CDC, parser_pkg
│   ├── Uart_Tx/                     #   TX: PHY, FSM, MAC FSM, Composer
│   ├── Sequencers/                  #   rx_msg_handler, burst/single seq, rgf_manager
│   ├── Memory/                      #   SRAM (R/G/B) + Async FIFO
│   ├── RGF/                         #   6 register file modules
│   └── Clock_Reset/                 #   Clock mux + reset synchronizer
│
├── Testbenches/                     # SystemVerilog simulation testbenches
│   ├── tb_uart_rx_phy_mac.sv        #   PHY + MAC unit test
│   ├── tb_uart_rx_full_chain.sv     #   PHY + MAC + Classifier chain test
│   ├── tb_uart_rx_system.sv         #   Full system integration test (with CDC)
│   └── ...
│
├── image_testing/                   # Python host-side scripts
│   ├── scripts/
│   │   ├── send_burst_image.py      #   Send 256x256 image to FPGA
│   │   ├── receive_burst_image.py   #   Read image back from FPGA
│   │   ├── test_rgf_readwrite.py    #   RGF register verification
│   │   ├── compare_images.py       #   Pixel-level image comparison
│   │   ├── test_uart_connection.py #   Basic UART connectivity test
│   │   ├── test_rx_simple.py       #   RX path diagnostic
│   │   ├── test_tx_loopback.py     #   TX loopback diagnostic
│   │   └── test_tx_trigger.py      #   TX trigger diagnostic
│   ├── img_input/                   #   Source test images
│   └── img_output/                  #   Images received from FPGA
│
├── Constraints/                     # Nexys A7-100T pin constraints (.xdc)
├── build/                           # FPGA bitstream (.bit)
├── simulation/                      # Waveform / ILA screenshots
└── docs/                            # Report, schematics, RGF specs
```

See [`docs/STRUCTURE.md`](docs/STRUCTURE.md) for the full annotated file tree.

---

## Getting Started

### Prerequisites

- **Vivado 2024.1+** (synthesis, implementation, bitstream generation)
- **Python 3.10+** with `pyserial` and `Pillow`
- **Nexys A7-100T** board connected via USB-UART

### Build & Program

1. Open Vivado, create a new project targeting **xc7a100tcsg324-1**
2. Add all `.sv` files under `rtl/` as design sources
3. Add `Constraints/Nexys-A7-100T-Master.xdc` as constraints
4. Run Synthesis, Implementation, and Generate Bitstream
5. Program the FPGA via Hardware Manager

Or use the pre-built bitstream:
```
Vivado Hardware Manager → Program Device → build/uart_top.bit
```

### Send an Image

```bash
pip install pyserial Pillow

cd image_testing/scripts
python send_burst_image.py          # Sends img_input/ image to FPGA
python receive_burst_image.py       # Reads image back to img_output/
```

### Verify Registers

```bash
python test_rgf_readwrite.py        # Writes & reads back all RGF registers
```

---

## RTL Module Summary

| Module | Domain | Description |
|--------|--------|-------------|
| `uart_top` | Both | Top-level: integrates all modules, CDC glitch guards, LED diagnostics |
| `uart_rx_PHY` | 176 MHz | Bit-level reception, 32x oversampling, triple-flop synchronizer |
| `uart_rx_MAC` | 176 MHz | Byte assembly, message framing, frame-safe delimiter handling |
| `uart_classifier` | 176 MHz | Message type decode, field extraction (addr, data, height, width, RGB) |
| `uart_cdc_sync` | Both | 2-FF/3-FF synchronizers, edge-triggered data sampling, aligned handshake |
| `rx_msg_handler` | 100 MHz | Unified RX handler wrapping sequencers + RGF manager |
| `seq_rx_image_burst` | 100 MHz | Burst write sequencer: pixel messages to SRAM (PC to FPGA) |
| `Sequencer_Burst_Tx` | 100 MHz | Burst read sequencer: SRAM to async FIFO (FPGA to PC) |
| `rgf_manager` | 100 MHz | RGF read/write state machine with persistent `rd_en` |
| `FIFO_Async` | Both | Gray-code async FIFO for TX path clock domain crossing |
| `msg_compposer` | 176 MHz | Formats RGF read responses and burst pixel TX messages |
| `uart_tx_mac_fsm` | 176 MHz | Byte-by-byte message dispatch with immediate data latch |
| `uart_tx` | 176 MHz | Serial bit transmission with baud-rate FSM |

---

## Simulation

Testbenches are located in `Testbenches/` and can be run in Vivado Simulator:

| Testbench | Scope |
|-----------|-------|
| `tb_uart_rx_phy_mac` | PHY + MAC byte reception |
| `tb_uart_rx_full_chain` | PHY + MAC + Classifier (all 6 message types) |
| `tb_uart_rx_system` | Full system: PHY through CDC to sequencers and RGF |

---

## Design Challenges Solved

- **Pixel-as-Delimiter Collision**: Pixel value `0x7D` (125) matches the `}` frame delimiter. Solved with position-aware `close_at_end` logic in the MAC that only accepts `}` at expected byte positions (5, 10, 15).

- **CDC Combinational Glitch**: Multi-bit state transitions in the 100 MHz sequencer produced brief glitches on `burst_done`, sampled by the 176 MHz CDC as spurious pulses. Solved by registering combinational signals before clock domain crossing.

- **SRAM Pipeline Latency**: 1-cycle SRAM read latency caused data loss in the TX burst sequencer. Solved with pipeline tracking (`sram_read_req_d1`) to prevent issuing new reads before the previous result is consumed.

- **Height/Width Overwrite on Pixel Messages**: The classifier outputs `height=0, width=0` for pixel messages, which propagated through CDC and zeroed `max_rows` in the burst sequencer. Solved by latching dimensions only on `MSG_START_BURST_WR`.

---

## Author

**Assaf Afriat**

---

## License

This project is part of an academic course. All rights reserved.
