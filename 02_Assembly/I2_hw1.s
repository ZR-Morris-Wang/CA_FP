.data
    n: .word 10
    
.text
.globl __start



FUNCTION:
    addi x11, x0, 4 
    addi x6, x0, 3
    addi x7, x0, 10
    addi x30, t0, 0 # record the initial input value

    addi sp, sp, -8 # save return address and new funtion input into stack
    sw x1, 4(sp)
    sw x10, 0(sp)
    
    bge x10, x11, L1 # to see if the input is greater or equal than 4
    addi sp, sp, 8

    addi x10, x0, 3    
#    beq x30, a0, exit # if recursive function isn't used (input doesn't change), then exit the function
#    addi a0, x0, 3 # if the input is smaller than 4, then return 3
    jalr x0, 0(x1)
    
  L1:

#    lw x31, 0(sp)  # load caller a0 value before it's changed

    srai x10, x10, 2
    jal x1, FUNCTION
    
    addi x29, x10, 0 # restore the caller a0 value
    
    lw x31, 0(sp) # poping from a stack to get to the upper stage
    lw x1, 4(sp)
    addi sp, sp, 8
        
    mul x10, x29, x6 #get the temporary output of any stage
    mul x31, x31, x7
    add x10, x10, x31
    add x10, x10, x6
 #   beq x30, x31, exit # if a0 appears the same as the initial input(the uppermost stage), then exit the function
    jalr x0,0(x1)
#  exit: 
      # Todo: Define your own function in HW1
    # You should store the output into x10
    

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall