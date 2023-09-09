*** Settings ***

Library    stm32l476x_UART    port=COM3    baudrate=115200    WITH NAME    stm

*** Test Cases ***
Test GetProcessData
    [Documentation]    Test the GetProcessData method
    ${processData}    stm.GetProcessData   delay=1
    Log To Console    ${processData}

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   temp

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   pressure

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   logo

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   height

Test ChangeDisplay
    [Documentation]    Test the ChangeDisplay method
    stm.ChangeDisplay   adc

Test ChangeProcessUnits
    [Documentation]    Test the ChangeProcessUnit method
    stm.ChangeProcessUnits SetTempC

Test ChangeProcessUnits
    [Documentation]    Test the ChangeProcessUnit method
    stm.ChangeProcessUnits SetTempF
