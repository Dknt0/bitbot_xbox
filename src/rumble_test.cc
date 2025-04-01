/**
 * Cpp joystick rumble control
 * Refer to https://github.com/atar-axis/xpadneo
 *
 * Dknt 2024.9
 */

/* hidraw test, bypassing the driver for rumble control
 * for Xbox compatible devices only, use at your own risk
 */

// sudo apt install libncurses-dev
// sudo ./hidraw /dev/hidraw2

#include <errno.h>
#include <fcntl.h>
#include <ncurses.h>  // Build text-based user interfaces (TUI)
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef unsigned char u8;

/// @brief Rumble command struct
static struct ff_pack_t {
  char cmd;  // Set to 0x03
  struct {
    u8 weak : 1;
    u8 strong : 1;
    u8 right : 1;
    u8 left : 1;
  } enable;  // 0 or 1
  struct {
    u8 left;
    u8 right;
    u8 strong;
    u8 weak;
  } strength;  // 0 ~ 100
  struct {
    u8 sustain_10ms;
    u8 release_10ms;
    u8 loop_count;  // 0 ~ 255
  } pulse;
} ff_pack;

// Curse window struct
WINDOW *input;   // Input window
WINDOW *output;  // Output window. Used to display old rumble command.

/// @brief Print Rumble command
/// @param window
static void print_pack(WINDOW *window) {
  if (window == output) {
    scroll(output);
    // Move curse
    wmove(output, LINES - 6, 0);
  }
  // Print new info
  // %02X is uppercase hexadecimal format with at least two digits
  wprintw(window,
          "%02X  %d   %u   %u   %u   0   0   0   0  %03d %03d %03d %03d  %03d "
          "%03d  %03d",
          ff_pack.cmd, ff_pack.enable.strong, ff_pack.enable.weak,
          ff_pack.enable.left, ff_pack.enable.right, ff_pack.strength.strong,
          ff_pack.strength.weak, ff_pack.strength.left, ff_pack.strength.right,
          ff_pack.pulse.sustain_10ms, ff_pack.pulse.release_10ms,
          ff_pack.pulse.loop_count);
}

/// @brief Print in input WINDOW
static void print_input() {
  // Move the cursor to a specified position
  wmove(input, 0, 0);
  // Print in window
  wprintw(
      input,
      "--[  MotorEnable  ]---------------[ Strength in %% ][  10ms ][cnt]--\n");
  print_pack(input);
  // Print in window
  wprintw(input,
          "\n"
          "    |   |   |   |                   |   |   |   |    |   |    |\n"
          "   STR WEA LTR RTR  -   -   -   -  STR WEA LTR RTR  SUS REL  LOP\n"
          "   [1] [2] [3] [4]                 q/a w/s e/d r/f  h/j k/l  u/i "
          "   <-- keys, or Enter = SEND, Ctrl+C = ABORT");
  // Refresh the window to show changes
  wrefresh(input);
}

int hidraw = -1;  // Low-level access to Human Interface Devices (HID),

int main(int argc, char **argv) {
  char ch;  // Used for wgetch

  // Initialize Rumble command struct
  ff_pack.cmd = 0x03;

  ff_pack.enable.strong = 1;
  ff_pack.enable.weak = 1;
  ff_pack.enable.left = 1;
  ff_pack.enable.right = 1;

  ff_pack.strength.strong = 40;
  ff_pack.strength.weak = 30;
  ff_pack.strength.left = 20;
  ff_pack.strength.right = 20;

  ff_pack.pulse.sustain_10ms = 5;
  ff_pack.pulse.release_10ms = 5;
  ff_pack.pulse.loop_count = 3;

  // Open a hidraw
  if (argc == 2) {
    hidraw = open(argv[1], O_WRONLY);
  } else {
    fprintf(stderr, "usage: %s /dev/hidraw##\n", argv[0]);
    exit(1);
  }

  // Check the hidraw
  if (hidraw < 0) {
    fprintf(stderr, "%s: error %d opening '%s': %s\n", argv[0], errno, argv[1],
            strerror(errno));
    exit(1);
  }

  // Start ncurses mode
  initscr();
  // Enter in raw mode, allows to disable line buffering
  // and special character processing. Including Ctrl-C...
  raw();
  // To enable the capture of special keys. stdscr is the hole terminal
  keypad(stdscr, TRUE);
  // Disable echoing of typed characters
  noecho();

  // Create window
  output = newwin(LINES - 5, COLS, 0, 0);  // Upper window
  input = newwin(5, COLS, LINES - 5, 0);   // Lower window

  // Print input information
  print_input();
  // Enable scrolling in a window when the cursor moves beyond the last line.
  scrollok(output, TRUE);

  // In ASC-II, Ctrl-C is 3
  while ((ch = wgetch(input)) != 3) {
    switch (ch) {
      case 10:  // Enter (\n) is 10
        // Print the old Rumble command in the output WINDOW
        print_pack(output);
        // Write rumble infomation into hidraw
        write(hidraw, &ff_pack, sizeof(ff_pack));
        // Refresh the output window to show changes
        wrefresh(output);
        break;
      case '1':
        ff_pack.enable.strong ^= 1;
        break;
      case '2':
        ff_pack.enable.weak ^= 1;
        break;
      case '3':
        ff_pack.enable.left ^= 1;
        break;
      case '4':
        ff_pack.enable.right ^= 1;
        break;
      case 'q':
        if (ff_pack.strength.strong < 100) ff_pack.strength.strong++;
        break;
      case 'a':
        if (ff_pack.strength.strong > 0) ff_pack.strength.strong--;
        break;
      case 'w':
        if (ff_pack.strength.weak < 100) ff_pack.strength.weak++;
        break;
      case 's':
        if (ff_pack.strength.weak > 0) ff_pack.strength.weak--;
        break;
      case 'e':
        if (ff_pack.strength.left < 100) ff_pack.strength.left++;
        break;
      case 'd':
        if (ff_pack.strength.left > 0) ff_pack.strength.left--;
        break;
      case 'r':
        if (ff_pack.strength.right < 100) ff_pack.strength.right++;
        break;
      case 'f':
        if (ff_pack.strength.right > 0) ff_pack.strength.right--;
        break;
      case 'h':
        if (ff_pack.pulse.sustain_10ms > 0) ff_pack.pulse.sustain_10ms--;
        break;
      case 'j':
        if (ff_pack.pulse.sustain_10ms < 255) ff_pack.pulse.sustain_10ms++;
        break;
      case 'k':
        if (ff_pack.pulse.release_10ms > 0) ff_pack.pulse.release_10ms--;
        break;
      case 'l':
        if (ff_pack.pulse.release_10ms < 255) ff_pack.pulse.release_10ms++;
        break;
      case 'u':
        if (ff_pack.pulse.loop_count > 0) ff_pack.pulse.loop_count--;
        break;
      case 'i':
        if (ff_pack.pulse.loop_count < 255) ff_pack.pulse.loop_count++;
        break;
    }
    // Print information
    print_input();
  }

  // Clean up WINDOW
  delwin(input);
  delwin(output);

  // End ncurses mode
  endwin();

  // Close hidraw
  close(hidraw);

  exit(0);
}
