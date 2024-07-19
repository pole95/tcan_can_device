# tcan_can_device

## Overview
A library to communicate with CAN devices, fork of [tcan](https://github.com/leggedrobotics/tcan).

This fork removes the use of a Bus as the main container to group different devices. Instead, each device is considered as independent, and opens its own socket to communicate on a can bus.

**Maintainer**: Pol Eyschen, peyschen@ethz.ch