# Fylo Internals

`pyhulax.fylo` contains the lower-level runtime protocol, command, and state
machinery that [`pyhulax`][pyhulax.DroneAPI] builds on top of.

It is packaged with the SDK, but it is primarily an internal layer rather than
the main end-user entrypoint.

## Control Server

::: pyhulax.fylo.controlserver
    options:
      show_if_no_docstring: true
      show_attribute_values: true

## Command Processor

::: pyhulax.fylo.commandprocessor
    options:
      show_if_no_docstring: true
      show_attribute_values: true

## State Processor

::: pyhulax.fylo.stateprocessor
    options:
      show_if_no_docstring: true
      show_attribute_values: true

## Task Processor

::: pyhulax.fylo.taskprocessor
    options:
      show_if_no_docstring: true
      show_attribute_values: true

## MAVLink Helpers

::: pyhulax.fylo.mavlink
    options:
      show_if_no_docstring: true
      show_attribute_values: true

## UWB Helpers

::: pyhulax.fylo.uwb
    options:
      show_if_no_docstring: true
      show_attribute_values: true
