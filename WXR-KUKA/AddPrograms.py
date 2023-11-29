from robodk.robolink import *

RDK = Robolink()

main_program = RDK.Item("MainProgram")

# Custom num
start_number = 16
end_number = 100

for number in range(start_number, end_number + 1):
    program_name = f"MyProgram{number:03d}"  # 3자리 숫자로 포매팅 (예: 001, 002, ...)
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)

    if program.Valid():
        main_program.RunCodeCustom(program.Name(), INSTRUCTION_CALL_PROGRAM)

RDK.ShowMessage(f"MyProgram{start_number:03d}부터 MyProgram{end_number:03d}까지 Main 프로그램에 추가되었습니다.", False)