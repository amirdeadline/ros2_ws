import time
import argparse
import I2C_LCD_driver

def display_message(line1, line2, duration):
    # Initialize the LCD
    lcd = I2C_LCD_driver.I2C_LCD_driver()
    lcd.lcd_init()

    # Display the first line of the message
    lcd.lcd_string(line1, I2C_LCD_driver.I2C_LCD_driver.LCD_LINE_1)
    
    # Display the second line of the message
    lcd.lcd_string(line2, I2C_LCD_driver.I2C_LCD_driver.LCD_LINE_2)
    
    # Keep the message displayed for the specified duration
    time.sleep(duration)
    
    # Clear the display
    lcd.lcd_byte(0x01, I2C_LCD_driver.I2C_LCD_driver.LCD_CMD)  # Clear display command

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Display a message on the I2C LCD.")
    parser.add_argument("-l1", "--line1", type=str, required=True, help="The message to display on line 1.")
    parser.add_argument("-l2", "--line2", type=str, required=True, help="The message to display on line 2.")
    parser.add_argument("-t", "--time", type=int, default=2, help="The duration to display the message (in seconds).")
    args = parser.parse_args()

    # Display the two-line message on the LCD
    display_message(args.line1, args.line2, args.time)
