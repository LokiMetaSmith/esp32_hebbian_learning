#ifndef SYNSENSE_MEMORY_MAP_H
#define SYNSENSE_MEMORY_MAP_H

// --- Xylo Core Memory Map (from Xylo Audio v3 Datasheet, Chapter 10) ---

// Note: These are 18-bit addresses.

#define IWTRAM_START_ADDR       0x00200
#define IWTRAM_END_ADDR         0x009FF

#define IWT2RAM_START_ADDR      0x00A00
#define IWT2RAM_END_ADDR        0x011FF

#define NSCRAM_START_ADDR       0x01200
#define NSCRAM_END_ADDR         0x015FF

#define HSC2RAM_START_ADDR      0x01600
#define HSC2RAM_END_ADDR        0x019DF

#define NMPRAM_START_ADDR       0x019E0
#define NMPRAM_END_ADDR         0x01DDF

#define NDSRAM_START_ADDR       0x01DE0
#define NDSRAM_END_ADDR         0x021DF

#define HDS2RAM_START_ADDR      0x021E0
#define HDS2RAM_END_ADDR        0x025BF

#define NDMRAM_START_ADDR       0x025C0
#define NDMRAM_END_ADDR         0x029BF

#define NBRAM_START_ADDR        0x029C0
#define NBRAM_END_ADDR          0x02DBF

#define NTHRAM_START_ADDR       0x02DC0
#define NTHRAM_END_ADDR         0x031BF

#define HCRAM_START_ADDR        0x031C0
#define HCRAM_END_ADDR          0x0359F

#define HARAM_START_ADDR        0x035A0
#define HARAM_END_ADDR          0x0397F

#define HSPKRAM_START_ADDR      0x03980
#define HSPKRAM_END_ADDR        0x03D5F

#define HEFOCRAM_START_ADDR     0x03D60
#define HEFOCRAM_END_ADDR       0x0413F

#define HFORAM_START_ADDR       0x04140
#define HFORAM_END_ADDR         0x0BD3F

#define HWTRAM_START_ADDR       0x0BD40
#define HWTRAM_END_ADDR         0x1393F

#define HWT2RAM_START_ADDR      0x13940
#define HWT2RAM_END_ADDR        0x1B53F

#define OWTRAM_START_ADDR       0x1B540
#define OWTRAM_END_ADDR         0x1C53F


#endif // SYNSENSE_MEMORY_MAP_H
