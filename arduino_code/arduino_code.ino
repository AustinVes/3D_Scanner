/*  This is a dummy file so that we can load the other .ino files before setup() and loop().
 *  Arduino allows multiple .ino files in a folder, all of which are stitched into one giant .ino by the pre-compiler.
 *  The pre-compiler starts with the .ino that has the same name as the parent folder (i.e. the "main" one),
 *  then tacks on each successive .ino in alphabetical order. Since our setup() and loop() depend on global variables defined
 *  other sections of the code, we left this file blank and instead controlled the order in which the various sections of code 
 *  were loaded by preceding each other filename with a letter.
 *  Though ugly, this is the simplest way we thought of to organize our code while handling overlapping dependencies with global
 *  variables, which would have been beyond our current ability to accomplish with header files.
 */
