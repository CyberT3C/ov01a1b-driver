DefinitionBlock ("ssdt-ircam.aml", "SSDT", 2, "CUSTOM", "IRCam", 0x00000001)
{
    // Externe Referenzen
    External (\_SB.PC00.IPU0, DeviceObj)
    External (\_SB.PC00.I2C2, DeviceObj)

    // Wir erstellen unseren Sensor-Knoten am I2C-Bus
    Scope (\_SB.PC00.I2C2)
    {
        Device (IRC0)
        {
            Name (_ADR, 0x10)
            Name (_CID, "INTC10A0")
            Name (_UID, "1")

            // Namensdefinition.
            Name (_DSD, Package ()
            {
                // UUID für V4L2-Graphen
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),

                // Port/Endpoint-Struktur 
                Package ()
                {
                    // Sensor Port 0
                    Package (0x02) { "port", 0 },

                    // Endpoint für Sensor Port 0
                    Package (0x02)
                    {
                        "endpoint",
                        Package ()
                        {
                            // UUID für die V4L2-Endpoint-Properties
                            ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                            
                            // Liste -> Properties
                            Package ()
                            {
                                // Jedes Property ist ein eigenes 2-Elemente-Paket
                                Package (0x02) { "bus-format", 0x200E },        // MEDIA_BUS_FMT_Y10_1X10
                                Package (0x02) { "data-lanes", Package(1){1} }, // 1 Lane
                                
                                // Merke! Ein QWord-Literal ist einfach die Zahl selbst.
                                Package (0x02) { "link-frequencies", Package(1){ 400000000 } },
                                
                                Package (0x02) { "clock-frequency", 19200000 },

                                // Der Link zum IPU-Port.
                                Package (0x02) { "remote-endpoint", \_SB.PC00.IPU0.PRT1 }
                            }
                        }
                    }
                }
            }) // Ende des _DSD-Pakets

            Method (_STA, 0, NotSerialized)  { Return (0x0F) }
        }
    }

    // Wir fügen der IPU einen Port hinzu, der als Ziel für unseren Sensor dient.
    Scope (\_SB.PC00.IPU0)
    {
        // Eine Vermutung, pray to RNGsus!
        Device (PRT1)
        {
            Name (_ADR, 1) // Port Nummer 1

            // _DSD für den IPU-Port (als statischer Name)
            Name (_DSD, Package ()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package (0x02) { "port", 1 },
                    Package (0x02) { "endpoint", \_SB.PC00.I2C2.IRC0 }
                }
            })
        }
    }
}
