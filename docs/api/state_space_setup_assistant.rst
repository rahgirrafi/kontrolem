state\_space\_setup\_assistant
==============================

The web wizard's backend. See the :doc:`guide <../guides/setup_assistant>`
for the step-by-step tour; the HTTP API is defined in ``app.create_app``.

app — Flask application and HTTP API
------------------------------------

.. automodule:: state_space_setup_assistant.app

session — wizard state
----------------------

.. automodule:: state_space_setup_assistant.session

introspect — URDF joint/link parsing
------------------------------------

.. automodule:: state_space_setup_assistant.introspect

validation — control-oriented URDF checks
-----------------------------------------

.. automodule:: state_space_setup_assistant.validation

equilibrium — automatic operating-point finder
----------------------------------------------

.. automodule:: state_space_setup_assistant.equilibrium

benchmark — controller comparison
---------------------------------

.. automodule:: state_space_setup_assistant.benchmark

response — closed-loop response step
------------------------------------

.. automodule:: state_space_setup_assistant.response

yamlgen — config file generation
--------------------------------

.. automodule:: state_space_setup_assistant.yamlgen

schemas — controller parameter metadata
---------------------------------------

.. automodule:: state_space_setup_assistant.schemas

export — artifact bundle writer
-------------------------------

.. automodule:: state_space_setup_assistant.export

resources — package:// resolution
---------------------------------

.. automodule:: state_space_setup_assistant.resources

cli — the ``ss_setup_assistant`` entry point
--------------------------------------------

.. automodule:: state_space_setup_assistant.cli
