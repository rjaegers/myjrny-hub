pub struct DisplayBuffer<'a> {
    pub buf: &'a mut [u16],
    pub width: i32,
    pub height: i32,
}

use embedded_graphics::{
    geometry,
    pixelcolor::{IntoStorage, Rgb565},
    Pixel,
};

// Implement DrawTarget for
impl embedded_graphics::draw_target::DrawTarget for DisplayBuffer<'_> {
    type Color = Rgb565;
    type Error = ();

    /// Draw a pixel
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            if point.x >= 0 && point.y >= 0 && point.x < self.width && point.y < self.height {
                let index = point.y * self.width + point.x;
                self.buf[index as usize] = color.into_storage();
            } else {
                // Ignore invalid points
            }
        }

        Ok(())
    }
}
impl geometry::OriginDimensions for DisplayBuffer<'_> {
    /// Return the size of the display
    fn size(&self) -> geometry::Size {
        geometry::Size::new(self.width as u32, self.height as u32)
    }
}
