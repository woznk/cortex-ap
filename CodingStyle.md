# Do’s #

**Development**

Create a branch, add some feature / fix a bug, compile, do unit test, merge into trunk.

**Coding**

Replace nested `if()` instructions with `switch()` instructions.

Always add the  `default :` branch to `switch ()` instructions.

Remove all “not reachable” code.

Remove all commented code.

Write one declaration and one instruction for each line.

**Names**

Use long names.

Separate words by underscores, never use leading underscore.

Static and global variables: initial capital letters

Function parameters, structure fields and automatic variables: lower case

Macros, structures, union and enumeration: uppercase

Add prefix STRUCT_, UNION_ and ENUM_._

```
#define MAX_ALTITUDE 19000 

typedef struct {
    uint8_t field_a;
    uint8_t field_b;
} STRUCT_SAMPLE;

uint8_t Module_Static_Variable;

void Function_A (uint8_t first_parameter, uint16_t second_parameter)
{
     uint8_t function_a_automatic_variable;
     …
}
```

**Comments**

Concise, explicit, complete.

Explain the meaning and function of every variable declaration.

Explain the return value of functions.

Comment language blocks, and any line that is not crystal clear.

**Functions**

Keep functions short.

Keep functions simple.

Break large functions into several smaller ones.

Document the meaning of the parameter in the comments.

Define a prototype for every called functions.

**Interrupt Service Routines**

Keep ISRs short.

Keep functions simple.

When an ISR grows too large or too slow, spawn another task / set a flag and
exit.

Budget time to carry out each ISR.

Never allocate or free memory in an ISR.

Never call functions inside ISRs.

Variables written inside ISR must never be written also outside ISR.

Point interrupt vector tables entries not used by the system to an error
handler.

**Nesting**

Nesting levels shall not be more than 4.

**Run-time check**

Check program flow.

Check array indexes.

Check over/underflow conditions after each arithmetic operation.

Check if divisor is zero before any division operation.

Check plausibility of results of arithmetic operations.

Check if parameters passed to functions contain valid data.

# Don’t’s #

**Coding**

Don’t use dynamic allocation.

Don’t use break instructions within loops.

Don’t use multiple entry-/ exit-points.

Don’t use goto.

Don’t use procedural parameters such as `Function_A(Function_B(x));`

Don’t use recursion.

Don’t use implicit conversion.

Don’t use common resources.

Don’t use magic numbers.

Limit use of global variables.

Limit use of pointers.

Don’t do assignments inside `if( )` conditions.

Don’t do multiple assignments as `a = b = 0xFF;`