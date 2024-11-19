# EBYTE E80-x00M2213S Module

This repository documents my study and implementation of the EBYTE E80-x00M2213S module.

## Project Contents

### Notes on DIO Pins
The following are some key DIO pin functions I've identified during my study:
- **DIO5/DIO6**: RF switch control pins. These pins are not exposed externally but need to be considered in software design for proper RF switching control.
- [**DIO9(PIN22)**](https://github.com/KunYi/EBYTE_E80_DEMO/blob/main/Core/Inc/main.h#L68C9-L68C27): Interrupt signal pin. This needs to be properly connected in hardware design for interrupt handling.
*Note: This is not a complete list of DIO functions. Please refer to the official documentation for full pin descriptions.*

### KiCAD v8 Resources
- [Module Footprint](KiCAD/E80-x00M2213S.kicad_mod)
- [Modificated STEP FILE](docs/E80-XXXM2213S_U.step), add pins

> **Important Note:** The dimensions between pin8/pin11 and pin16/pin19 are shown as 11.17mm in the official STEP model, which differs from the 11.20mm specified in the user manual. Please verify these measurements with your actual module before designing your PCB, as I haven't physically measured the module myself.

### Footprint Preview
![KiCAD Footprint Preview](KiCAD/kicad_footprint_review_new.png)
## Features
- Custom KiCAD footprint design
- Pin spacing based on official STEP model
- Compatible with KiCAD version 8

## Documentation and Resources

### Official Documentation
- [Product Page: EBYTE E80-900M2213S LR1121 LoRa Module](https://www.cdebyte.com/products/E80-900M2213S)
- [User Manual (English)](https://www.cdebyte.com/pdf-down.aspx?id=3188)
- [3D Model (STEP Format)](https://www.cdebyte.com/pdf-down.aspx?id=3370)

## License
This project is open-source. Feel free to use and modify the KiCAD footprint according to your needs.

The demo source code, user manual, and STEP model are intellectual property of EBYTE. Please refer to EBYTE's official website for their usage terms and conditions.

## Contributing
Contributions, issues, and feature requests are welcome. Feel free to check the issues page if you want to contribute.
