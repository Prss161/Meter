*** Settings ***

Library    stm32l476x_UART    port=COM3    baudrate=115200    WITH NAME    stm  # Assuming you save your class in stm32l476x.py

*** Test Cases ***
Test GetProcessData
    [Documentation]    Test the GetProcessData method
    ${processData}    stm.GetProcessData   delay=1
    Log To Console    ${processData}

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   temp