# How to test acting components

**Summary:** todo

---

## Author

Julian Graf

## Date

08.01.2023

<!-- TOC -->
* [How to test acting components](#how-to-test-acting-components)
  * [Author](#author)
  * [Date](#date)
  * [velocity_controller](#velocitycontroller)
<!-- TOC -->

## velocity_controller

To test the velocity controller, set the parameter ```enabled``` in the acting launch-file true. Now the velocity_publisher_dummy will publish dummy target speeds. Use ```rqt_plot /carla/hero/Speed /carla/hero/max_velocity``` to visualize target and current speeds.
