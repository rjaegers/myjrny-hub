use embedded_graphics::{
    geometry,
    pixelcolor::{IntoStorage, Rgb565},
    primitives::Rectangle,
    Pixel,
};

pub struct DisplayBuffer<'a> {
    pub buf: &'a mut [u16],
    pub width: i32,
    pub height: i32,
}

// Implement DrawTarget for
impl embedded_graphics::draw_target::DrawTarget for DisplayBuffer<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

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
            }
        }

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
        //Ok(())
    }
}

impl geometry::OriginDimensions for DisplayBuffer<'_> {
    fn size(&self) -> geometry::Size {
        geometry::Size::new(self.width as u32, self.height as u32)
    }
}

#[derive(Copy, Clone)]
pub enum Rotation {
    Rotate0,
    Rotate90,
    Rotate180,
    Rotate270,
}

pub struct RotatedDisplayBuffer<'a> {
    pub inner: DisplayBuffer<'a>,
    pub rotation: Rotation,
}

fn rotate_point(
    point: geometry::Point,
    rotation: Rotation,
    width: i32,
    height: i32,
) -> geometry::Point {
    match rotation {
        Rotation::Rotate0 => point,
        Rotation::Rotate90 => geometry::Point::new(height - 1 - point.y, point.x),
        Rotation::Rotate180 => geometry::Point::new(width - 1 - point.x, height - 1 - point.y),
        Rotation::Rotate270 => geometry::Point::new(point.y, width - 1 - point.x),
    }
}

impl embedded_graphics::draw_target::DrawTarget for RotatedDisplayBuffer<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            let (width, height) = (self.inner.width, self.inner.height);
            let rotated_point = rotate_point(point, self.rotation, width, height);

            if rotated_point.x >= 0
                && rotated_point.y >= 0
                && rotated_point.x < width
                && rotated_point.y < height
            {
                if let Some(index) = rotated_point
                    .y
                    .checked_mul(width)
                    .and_then(|v| v.checked_add(rotated_point.x))
                {
                    self.inner.buf[index as usize] = color.into_storage();
                }
            }
        }

        Ok(())
    }
}

impl geometry::OriginDimensions for RotatedDisplayBuffer<'_> {
    fn size(&self) -> geometry::Size {
        match self.rotation {
            Rotation::Rotate90 | Rotation::Rotate270 => {
                geometry::Size::new(self.inner.height as u32, self.inner.width as u32)
            }
            Rotation::Rotate0 | Rotation::Rotate180 => {
                geometry::Size::new(self.inner.width as u32, self.inner.height as u32)
            }
        }
    }
}
