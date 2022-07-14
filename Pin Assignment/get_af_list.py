from pyb import Pin  # type:ignore
import os

pin_list1 = [
    "B13",
    "B14",
    "B11",
    "B12",
    "E15",
    "B10",
    "E13",
    "E14",
    "E11",
    "E12",
    "E9",
    "E10",
    "E7",
    "E8",
    "B0",
    "B1",
    "C4",
    "C5",
    "A6",
    "A7",
    "A4",
    "A5",
    "A2",
    "A3",
    "A0",
    "A1",
    "C2",
    "C3",
    "C0",
    "C1",
    "E6",
    "C13",
    "E4",
    "E5",
    "E2",
    "E3",
]

pin_list2 = [
    "B15",
    "D8",
    "D9",
    "D10",
    "D11",
    "D12",
    "D13",
    "D14",
    "D15",
    "C6",
    "C7",
    "C8",
    "C9",
    "A8",
    "A9",
    "A10",
    "A11",
    "A12",
    "A15",
    "C10",
    "C11",
    "C12",
    "D0",
    "D1",
    "D2",
    "D3",
    "D4",
    "D5",
    "D6",
    "D7",
    "B3",
    "B5",
    "B6",
    "B7",
    "B8",
    "B9",
    "E0",
    "E1",
]


f = open("pinlist.txt", "w")

for i in range(len(pin_list1)):
    ls = Pin(pin_list1[i])
    if len(ls.af_list()) >= 1:
        f.write("{} : {}\n".format(pin_list1[i], ls.af_list()))

for i in range(len(pin_list2)):
    ls = Pin(pin_list2[i])
    if len(ls.af_list()) >= 1:
        f.write("{} : {}\n".format(pin_list2[i], ls.af_list()))

f.close()
