# 🧩 MCUDecode / MCUSplice

Two small Python tools for working directly on the entropy-coded data of JPEG files.

- 🔍 **MCUDecode.py** decodes the Huffman scan and logs the exact bit offset of every MCU.
- ✂️ **MCUSplice.py** inserts or deletes MCUs in a JPEG, or shifts their colour, without changing the JPEG structure (headers, tables, image size, MCU count and restart interval stay the same).

🐍 Pure Python 3.8+ standard library. No dependencies.

## 📁 Supported files

✅ Baseline and extended sequential Huffman JPEG (SOF0 / SOF1). ❌ Progressive, arithmetic and lossless JPEG are rejected. MCUSplice additionally requires a single interleaved scan.

---

## 🔍 MCUDecode

```
python MCUDecode.py image.jpg [log_dir] [--adv]
```

| Argument | Meaning |
|---|---|
| `image.jpg` | Input file. |
| `log_dir` | Where to write the logs. Default: the folder of the input file. |
| `--adv` | Keep decoding MCUs until `FFD9`, ignoring the size in the header. Useful when the file holds more data than the header says. |

Output:

- `Log_MCUs.txt` for the main image.
- `Log_MCUs_Thumb.txt` for the EXIF thumbnail, if there is one.

### 📝 Log format

Offsets are decimal `<byte>.<bit>`, counted from the start of the file. Bit 0 is the MSB of the byte. The byte part is a real file offset, so byte-stuffing (`FF 00`) is already accounted for.

```
// 3000x2000 3 components:
// ID:1 [22] Dest:0
// ID:2 [11] Dest:1
// ID:3 [11] Dest:1
// MCU: YYYYCC (16x16 pixel)
//[188x125=23500 MCU]


MCU 0 (0,0) (623.0)
MCU 1 (1,0) (655.5)
*** [Y@701.3] Huffman error -> AC:0 #2
```

- `MCU n (x,y) (byte.bit)` is the start of MCU `n` at MCU grid position `(x,y)`.
- `*** [...]` lines mark errors: `Huffman error`, `Coefficient overflow`, `End of data`, or a missing RST marker. The label is the block type (`Y` or `C`) and its offset. `#n` is the block index inside the MCU.

### 🩹 Behaviour on damaged files

- 🚧 A stray marker (`FF xx`, `xx != 00`) inside the scan is treated as corrupt data. Decoding continues.
- 🔄 If an expected RST marker is missing, the decoder logs it and resynchronises on the next RST marker it finds.
- 📌 A note is added to the log when bytes remain before `FFD9` after the last MCU of the header. Run with `--adv` to decode them.

⚡ Speed: about 1 second for a 3000x2000 image (3.8 MB, 23,500 MCU).

---

## ✂️ MCUSplice

```
python MCUSplice.py in.jpg [out.jpg] [{ins|del}N][@x,y] [yV] [cbV] [crV] [yccY,Cb,Cr]
```

The output file is the second file argument. Default: `<name>_out.jpg`.

### ➕➖ Insert and delete

| Command | Effect |
|---|---|
| `ins10` | Insert 10 neutral MCU at MCU (0,0). |
| `del20` | Delete 20 MCU starting at MCU (0,0). |
| `ins20@10,50` | Insert 20 neutral MCU before MCU x=10, y=50. |
| `del20@10,50` | Delete 20 MCU starting at MCU x=10, y=50. |

MCU order is raster order: `index = y * MCUs_per_row + x`.

🔒 To keep the structure unchanged, the total MCU count never changes:

- ⬇️ **Insert:** the tail is pushed out of the image and cut off.
- ⬆️ **Delete:** the tail is padded with neutral MCUs.
- A neutral MCU has every block coded as DC difference 0 followed by EOB, so it is flat and keeps the previous DC level.
- RST markers are rebuilt using the original DRI.

### ⚙️ How the cut is done

For `ins` and `del` without colour options, MCUSplice works from `Log_MCUs.txt`, which sits in the same folder as the input file:

1. If the log is missing, or does not match the image (MCU grid in the header and offset of MCU 0), MCUSplice runs `MCUDecode.py <input>` automatically. `MCUDecode.py` must be in the same folder as `MCUSplice.py`.
2. It reads the bit range of each MCU from the log and copies or removes those bits directly. Nothing is decoded again.

🌈 DC values are stored as differences. Since the bits are copied as they are, MCUs after the cut point can show a colour shift until the next RST marker, or to the end of the image when DRI is 0.

⚠️ The log must belong to the input file. If you edit the image and splice again, delete the old `Log_MCUs.txt` or let the grid check replace it.

### 🎨 Colour adjust

Options go after `ins` / `del`.

| Option | Meaning |
|---|---|
| `y30 cb-20 cr15` | Relative. Adds to the DC value, in pixel level. An unsigned number also adds. |
| `ycc128,0,0` | Absolute. Y=128 is mid grey, Cb=0 and Cr=0 are neutral. Use `_` to skip a channel. |

What gets adjusted:

- ➕ With `ins`: exactly the N MCU just inserted. The base level is the DC of the MCU before them.
- ➖ With `del`: the N MCU right after the deleted range. AC is kept, only DC changes.
- 🖼️ With neither: every MCU from `@x,y` (default `@0,0`) to the end of the image.

Colour adjust decodes the DC values and encodes them again, so it does not use the log. It needs the DC Huffman table to contain a code for each category it writes. Files with an optimised table may fail with `DC Huffman table has no code for category N`.

### 💡 Examples

```
python MCUSplice.py a.jpg del3@2,1
python MCUSplice.py a.jpg out.jpg ins10@10,50
python MCUSplice.py a.jpg y30 @10,50
python MCUSplice.py a.jpg cb-10
python MCUSplice.py a.jpg out.jpg del5@4,2 ycc128,0,0
```

Typical output:

```
160x96 MCU 10x6=60 DRI=0 | del 3 @2,1
Saved: a_out.jpg
```

---

## 📄 License

GNU General Public License, version 2 or (at your option) any later version.
Copyright (C) 2026 Vũ Quang Đại
