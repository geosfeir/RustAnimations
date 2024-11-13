use smart_leds::RGB8;

pub const WIDTH : usize = 8;
pub const HEIGHT : usize = 8;
pub const NUM_PX : usize = WIDTH*HEIGHT;

// pulse implementation
pub struct Bright {
    strip: [RGB8; NUM_PX],
    color: RGB8,
    px_counter: u8,
    descending: bool,
}

impl Bright {
    // constructor fn
    pub fn new(color: RGB8) -> Bright {
        Self {
            strip: [RGB8::new(0,0,0); NUM_PX],
            color: color,
            px_counter: 0,
            descending: false,
        }
    }

    // turn off all LEDs
    pub fn clear(&mut self) {
        for px in self.strip.iter_mut() {
            *px = RGB8::new(0,0,0);
        }
    }

    // set an LED color
    pub fn set(&mut self, color: RGB8) {
        for px in self.strip.iter_mut() {
            *px = color;
        }
    }

    pub fn to_list(&self) -> [RGB8; NUM_PX] {
        self.strip
    }

    pub fn next(&mut self) {
        let brightness: u8 = if self.descending {
            200 - self.px_counter
        } else {
            self.px_counter
        };

        self.set(RGB8::new(
            (self.color.r as u16 * brightness as u16 / 200) as u8,
            (self.color.g as u16 * brightness as u16 / 200) as u8,
            (self.color.b as u16 * brightness as u16 / 200) as u8,
        ));

        // Update px_counter to create a pulsing effect
        if self.descending {
            if self.px_counter <= 0 {
                self.descending = false;
            } else {
                self.px_counter -= 5; // Decrease at a fixed rate
            }
        } else {
            if self.px_counter >= 200 {
                self.descending = true;
            } else {
                self.px_counter += 10; // Increase at a fixed rate
            }
        }
    }
}

// sprial implementation
pub struct Snake
{
    strip: [RGB8; NUM_PX],
    color: RGB8,
    delta: bool,
    row: usize,
    col: usize,
}

impl Snake {
    // constructor
    pub fn new(color: RGB8) -> Snake {
        Self {
            strip: [RGB8::new(0,0,0); NUM_PX],
            color: color,
            delta: true,
            row: 0,
            col: 0,
        }
    }

    // set pixels at (row,col)
    pub fn set(&mut self){
        for (idx, px) in self.strip.iter_mut().enumerate() {
            if idx == self.col*WIDTH + self.row {
                *px = self.color;
            } else {
                *px = RGB8::new(0,0,0);
            }
        }
    }

    pub fn to_list(&self) -> [RGB8; NUM_PX] {
        self.strip
    }

    pub fn next(&mut self) {
        // bounce the row value
        if self.row == WIDTH-1 {
            self.delta = false;
            self.col = (self.col + 1) % 8;
        } else if self.row == 0 {
            self.delta = true;
            self.col = (self.col + 1) % 8;
        }
        if self.delta { self.row += 1 } else { self.row -= 1 };
        // update
        self.set();
    }

}

pub struct Spiral {
    strip: [RGB8; NUM_PX],
    color: RGB8,
    index: usize,
    layer: usize,
}

impl Spiral {
    pub fn new(color: RGB8) -> Spiral {
        Self {
            strip: [RGB8::new(0, 0, 0); NUM_PX],
            color,
            index: 0,
            layer: 0,
        }
    }

    pub fn clear(&mut self) {
        for px in self.strip.iter_mut() {
            *px = RGB8::new(0, 0, 0);
        }
    }

    pub fn to_list(&self) -> [RGB8; NUM_PX] {
        self.strip
    }

    fn xy_to_index(x: usize, y: usize) -> usize {
        y * WIDTH + x
    }

    pub fn next(&mut self) {
        self.clear(); // Clear LEDs on each step
        
        // Calculate boundaries for the current layer
        let max_x = WIDTH - 1 - self.layer;
        let max_y = HEIGHT - 1 - self.layer;
        let min_x = self.layer;
        let min_y = self.layer;

        // Check the bounds to make sure the layer is within the matrix
        if min_x > max_x || min_y > max_y {
            // Reset to start again if we've spiraled inward completely
            self.index = 0;
            self.layer = 0;
            return;
        }

        // Set LED for each segment of the current layer border
        let pos = self.index % (2 * (max_x - min_x + max_y - min_y));

        // Top row
        if pos < max_x - min_x + 1 {
            self.strip[Self::xy_to_index(min_x + pos, min_y)] = self.color;
        }
        // Right column
        else if pos < max_x - min_x + 1 + max_y - min_y {
            self.strip[Self::xy_to_index(max_x, min_y + pos - (max_x - min_x + 1))] = self.color;
        }
        // Bottom row
        else if pos < 2 * (max_x - min_x) + max_y - min_y + 1 {
            self.strip[Self::xy_to_index(
                max_x - (pos - (max_x - min_x + 1 + max_y - min_y)),
                max_y,
            )] = self.color;
        }
        // Left column
        else {
            self.strip[Self::xy_to_index(
                min_x,
                max_y - (pos - (2 * (max_x - min_x) + max_y - min_y + 1)),
            )] = self.color;
        }

        self.index += 1;

        // If we've completed a full loop around the border, go inward
        if self.index >= 2 * (max_x - min_x + max_y - min_y) {
            self.layer += 1;
            self.index = 0;
        }
    }
}

pub struct Bouncy {
    strip: [RGB8; NUM_PX],
    x: usize,
    y: usize,
    dx: isize,
    dy: isize,
    color: RGB8,
}

impl Bouncy {
    // constructor fn
    pub fn new(color: RGB8) -> Bouncy {
        Self {
            strip: [RGB8::new(0, 0, 0); NUM_PX],
            x: WIDTH / 2,
            y: HEIGHT / 2,
            dx: 1,
            dy: 1,
            color,
        }
    }

    // clear the LED matrix (set all LEDs to off)
    pub fn clear(&mut self) {
        for px in self.strip.iter_mut() {
            *px = RGB8::new(0, 0, 0);
        }
    }

    // set a specific LED at (x, y) to the current color
    fn set_pixel(&mut self, x: usize, y: usize, color: RGB8) {
        let idx = y * WIDTH + x;
        self.strip[idx] = color;
    }

    // update the animation to the next frame
    pub fn next(&mut self) {
        // Clear the current position
        self.clear();

        // Set the new position of the "ball"
        self.set_pixel(self.x, self.y, self.color);

        // Update position based on direction
        self.x = (self.x as isize + self.dx) as usize;
        self.y = (self.y as isize + self.dy) as usize;

        // Bounce off the walls by reversing direction when the ball hits the edge
        if self.x == 0 || self.x == WIDTH - 1 {
            self.dx = -self.dx;
        }

        if self.y == 0 || self.y == HEIGHT - 1 {
            self.dy = -self.dy;
        }
    }

    // return the LED strip as an array
    pub fn to_list(&self) -> [RGB8; NUM_PX] {
        self.strip
    }
}