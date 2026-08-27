# Alliance ordinary-vehicle lightbar EKF port

`AllianceLightbarPredictor` is an adaptation of the ordinary-vehicle `RobotModel` design in
`Alliance-Algorithm/rmcs_auto_aim_v2` (Alliance Algorithm Team, Nanjing University of Science and Technology).
The port keeps this project's coordinate conventions and public fire-control DTOs while preserving the important model semantics:

- 11D body state: center xyz, linear velocity xyz, body yaw/angular speed, forward/lateral radii, lateral height offset.
- Four-armour rigid geometry and eight predicted lightbars.
- Camera visibility gating.
- Complete-armour anchor association followed by horizontal lightbar-ID propagation.
- Per-lightbar 4D image observation `(upper_u, upper_v, lower_u, lower_v)`.
- Projection/geometry Jacobian, Joseph-form covariance update, and Alliance default Q/R/P values.
- PnP is used to seed the first body state, not as the continuing ordinary-vehicle EKF measurement.

Upstream source: `https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2`

## MIT License notice

Copyright (c) 2025 Alliance Algorithm Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
