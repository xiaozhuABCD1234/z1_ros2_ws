# Vendored Unitree code

`arm_common.h` in this directory is **not** ours. It is a verbatim copy of the Z1
wire-protocol definitions published by Unitree Robotics, vendored here so that
this workspace does not have to link against Unitree's prebuilt shared objects.

| | |
|---|---|
| Upstream repository | https://github.com/unitreerobotics/z1_sdk |
| File | `include/unitree_arm_sdk/message/arm_common.h` |
| Commit | `f1af2b42f2a39a5010946e049ad3fe324e2e6f06` (2024-10-29) |
| License | BSD 3-Clause (`LICENSE` in this directory) |
| Local modifications | **none** — byte-for-byte identical |

## Why the `z1_sdk` copy and not the `z1_controller` one

Both repositories ship an `arm_common.h`. They are *not* identical, and the
`z1_sdk` version is the better one to vendor:

* it is wrapped in `namespace UNITREE_ARM`, so it cannot collide with the
  workspace's own headers;
* it omits `JointStateOld` and the `UDPSendCmd` union, which describe the
  **MCU-facing** packet that only `z1_ctrl` needs — we never build those;
* its comment block for `Motor_State::error` documents one more bit (`0x40`).

The structures we actually use — `ArmFSMState`, `JointCmd`, `Motor_State`,
`JointState`, `Posture`, `TrajCmd`, `ValueUnion`, `SendCmd`, `RecvState` — are
**identical** in both repositories, so vendoring either is protocol-correct.

## What this header is used for

`z1_ros2_control/Z1System` is a *client* of the official `z1_controller`
(`z1_ctrl`) process. It speaks the same 147-byte `SendCmd` / 208-byte
`RecvState` datagrams over UDP on loopback that the official `z1_sdk` speaks.
Upstream never published a C++ header for "the client side of that link" as a
standalone, so this copy of `arm_common.h` is the authoritative definition.

Because both sides of the link are byte layouts on a wire, `z1_protocol.hpp`
pins every size and field offset down with `static_assert`, so a future upstream
change to this layout fails the build instead of silently corrupting traffic.

## Provenance of the field semantics

Sizes and offsets are asserted at compile time. The *runtime rules* around the
packet — which port to bind, that the receive buffer must be exactly 208 bytes,
the `0xFE 0xFF` header, and the fact that `z1_ctrl` forces `PASSIVE` when the
client stops sending — were recovered by disassembling
`libZ1_x86_64.so` (`ARMSDK::_sendRecv`, `ARMSDK::getSendCmd`) and
`libZ1_SDK_x86_64.so` (`CtrlComponents::CtrlComponents`). See
`z1_ros2_control/README.md` for the evidence and for what has *not* been
verified on hardware.

## License obligations

BSD 3-Clause permits copying and redistribution, including in binary form,
provided that the copyright notice, the list of conditions and the disclaimer
are retained, and that the Unitree name is not used to endorse this work.
`LICENSE` next to this file retains them. Keep the two files together.
