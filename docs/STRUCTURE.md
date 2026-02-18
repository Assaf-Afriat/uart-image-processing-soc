# UART Image Processing SoC - RTL Design

Multi-clock UART SoC for real-time 256x256 image transfer between PC and FPGA (Nexys A7-100T) at 5.5 Mbaud.

## Project Structure

```
Final Project v00/
|
|-- rtl/                         # All synthesizable RTL
|   |-- uart_top.sv              #   Top-level module (integrates all sub-modules)
|   |-- uart_types.sv            #   Shared type definitions
|   |
|   |-- Uart_Rx/                 #   UART Receive Path (176 MHz domain)
|   |   |-- uart_rx_PHY.sv       #     PHY: bit-level serial reception, 32x oversampling
|   |   |-- uart_rx_MAC.sv       #     MAC: byte assembly, message framing & validation
|   |   |-- uart_classifier.sv   #     Classifier: message parsing, field extraction
|   |   |-- uart_cdc_sync.sv     #     CDC: 176 MHz <-> 100 MHz clock domain crossing
|   |   |-- parser_pkg.sv        #     Package: message type enums & constants
|   |
|   |-- Uart_Tx/                 #   UART Transmit Path (176 MHz domain)
|   |   |-- uart_tx.sv           #     TX PHY: serial bit transmission
|   |   |-- uart_tx_fsm.sv       #     TX FSM: baud-rate timing & shift control
|   |   |-- uart_tx_mac_fsm.sv   #     TX MAC: byte-by-byte message dispatch
|   |   |-- msg_compposer.sv     #     Composer: formats response messages
|   |
|   |-- Sequencers/              #   Data Flow Managers (100 MHz domain)
|   |   |-- rx_msg_handler.sv    #     Unified RX handler (wraps sequencers + RGF manager)
|   |   |-- seq_rx_image_single.sv  #  Single-pixel write sequencer
|   |   |-- seq_rx_image_burst.sv   #  Burst image write sequencer (PC -> SRAM)
|   |   |-- Sequencer_Burst_Tx.sv   #  Burst image read sequencer  (SRAM -> PC)
|   |   |-- rgf_manager.sv       #     RGF read/write orchestration
|   |
|   |-- Memory/                  #   Storage Elements
|   |   |-- sram_red.sv          #     Red channel SRAM   (14-bit addr, 32-bit data)
|   |   |-- sram_green.sv        #     Green channel SRAM
|   |   |-- sram_blue.sv         #     Blue channel SRAM
|   |   |-- FIFO_Async.sv        #     Gray-code async FIFO (TX path CDC)
|   |
|   |-- RGF/                     #   Register File Modules (100 MHz domain)
|   |   |-- RGF_LED.sv           #     LED control register
|   |   |-- RGF_SYS.sv           #     System configuration register
|   |   |-- RGF_CNT.sv           #     Counter/status register
|   |   |-- RGF_PWM.sv           #     PWM control register
|   |   |-- RGF_IMG.sv           #     Image parameter register
|   |   |-- RGF_Seq_TX_IMG_FIFO.sv #   TX image FIFO control register
|   |
|   |-- Clock_Reset/             #   Clock & Reset Infrastructure
|       |-- glitchless_clock_mux.sv # Glitch-free clock multiplexer
|       |-- reset_synchronizer.sv   # Async reset synchronizer
|
|-- Testbenches/             # Simulation Testbenches
|   |-- tb_uart_rx_phy_mac.sv       # PHY + MAC unit test
|   |-- tb_uart_rx_full_chain.sv    # PHY + MAC + Classifier chain test
|   |-- tb_uart_rx_system.sv        # Full system integration test
|   |-- tb_RAM_seq_composer.sv      # RAM + sequencer + composer test
|   |-- tb_RAM_seq_composer_MAC.sv  # RAM + sequencer + composer + MAC test
|   |-- tb_uart_test_check.sv       # Basic UART check
|
|-- Constraints/             # FPGA Constraints
|   |-- Nexys-A7-100T-Master.xdc
|
|-- image_testing/           # Python Host Scripts & Test Images
|   |-- scripts/
|   |   |-- send_burst_image.py     # Send image PC -> FPGA (burst write)
|   |   |-- receive_burst_image.py  # Receive image FPGA -> PC (burst read)
|   |   |-- test_rgf_readwrite.py   # RGF register write/read verification
|   |   |-- compare_images.py       # Compare input vs output images
|   |   |-- test_uart_connection.py # Basic UART connectivity test
|   |   |-- test_rx_simple.py       # Simple RX path test
|   |   |-- test_tx_loopback.py     # TX loopback test
|   |   |-- test_tx_trigger.py      # TX trigger test
|   |-- img_input/                  # Source images for testing
|   |-- img_output/                 # Received images from FPGA
|
|-- build/                   # Build Artifacts
|   |-- uart_top.bit         # FPGA bitstream
|
|-- simulation/              # Simulation Waveform Captures
|   |-- *.png                # ILA / waveform screenshots
|
|-- docs/                    # Documentation & Reference
|   |-- Final Project Report.pdf       # Project report
|   |-- Modules and IO Final Project.xlsx  # Module I/O specification
|   |-- schematic.pdf                  # Board schematic
|   |-- rgf_specs/                     # Register file specifications
|       |-- RegsFile.xlsx              #   Master register map
|       |-- RGF_LED.xlsx               #   LED register spec
|       |-- RGF_SYS.xlsx               #   System register spec
|       |-- RGF_CNT.xlsx               #   Counter register spec
|       |-- RGF_PWM.xlsx               #   PWM register spec
|       |-- RGF_IMG.xlsx               #   Image register spec
|       |-- RGF_Seq_TX_IMG_FIFO.xlsx   #   TX FIFO register spec
```

## Clock Domains

| Domain   | Frequency | Modules                                    |
|----------|----------:|--------------------------------------------|
| Fast CLK |   176 MHz | RX PHY, RX MAC, Classifier, CDC, TX chain  |
| Sys CLK  |   100 MHz | Sequencers, RGF, SRAM, rx_msg_handler      |

## Message Protocol

All messages are ASCII-framed: `{ ... , ... , ... }`

| Type                  | Format (16 bytes unless noted)                |
|-----------------------|-----------------------------------------------|
| RGF Write             | `{W<addr><off_h><off_l>,V<dh1><dh0><pad>,V<dl1><dl0><pad>}` |
| RGF Read Request      | `{R<addr><off_h><off_l>}` (6 bytes)           |
| RGF Read Response     | `{R<addr><off_h><off_l>,V<pad><d31:24><d23:16>,V<pad><d15:8><d7:0>}` |
| Start Burst Write     | `{I<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}` |
| Burst Pixel           | `{R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}`   |
| Start Burst Read      | `{R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}` |

## Target

- **Board**: Digilent Nexys A7-100T (Artix-7 XC7A100T)
- **Baud Rate**: 5,500,000 (5.5 Mbaud)
- **Image Size**: 256 x 256 pixels (RGB)
- **SRAM**: 3 x 16K x 32-bit (R/G/B channels)
