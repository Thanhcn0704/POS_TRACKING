"""vision_pi5 — Raspberry Pi 5 vision pick-and-place application.

Layered by concern:
    config       constants / tunables (single source of truth)
    hardware/    camera + UART drivers (I/O)
    comms/       wire protocols (TCP RobotLink, UART frame format)
    vision/      detection, shape classification, coordinate geometry
    processing/  trajectory decision + robot-time prediction (math)
    tracking/    object identity (new vs old)
    pipeline/    threaded stage orchestration (detect / sender / display)
    calibration/ interactive setup tools
    main         entrypoint / wiring

Run:  python -m vision_pi5.main  [--no-display]
"""
