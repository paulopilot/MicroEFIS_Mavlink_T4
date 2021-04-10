#ifndef SPRITE_H__
#define SPRITE_H__

//#include <ILI9341_t3n.h>

#define GND 0x0000
#define WHI ILI9341_WHITE //0xFFFF
#define GRY ILI9341_LIGHTGREY //0xC618
#define GRN ILI9341_GREEN //0x07E0
#define DGN 0x05E0
#define BLK ILI9341_BLACK

#define GAUGE_LENGTH  39

const PROGMEM uint16_t needle_image[] = { \
                                          WHI, WHI, WHI, \
                                          WHI, WHI, WHI, \
                                          WHI, WHI, WHI, \
                                          DGN, WHI, DGN
                                        };

const PROGMEM uint16_t double_needle_image[] = { \
                                                 WHI, WHI, WHI, \
                                                 WHI, WHI, WHI, \
                                                 WHI, WHI, WHI, \
                                                 DGN, WHI, DGN, \
                                                 GRN, GRN, GRN, \
                                                 DGN, WHI, DGN, \
                                                 WHI, WHI, WHI, \
                                                 WHI, WHI, WHI, \
                                                 WHI, WHI, WHI\
                                               };

typedef struct  {
  // Location of top left corner.
  uint16_t x;
  uint16_t y;
  // Dimensions of sprite
  uint16_t width;
  uint16_t height;
  // The pixels for the sprite are stored at this location
  const uint16_t *SpriteImage;
  uint8_t visible;
} _sprite_struct;

class Sprite
{
  public:
    Sprite();
    void initSprite(ILI9341_t3n *tft, uint16_t x, uint16_t y, uint16_t minValue, uint16_t maxValue, bool _double =  false);
    void showSprite();
    void hideSprite();
    void drawSprite();
    void moveSprite(uint16_t x, uint16_t y);
    void moveSprite(uint16_t value);
    uint8_t withinSprite(uint16_t x, uint16_t y);
    void setMax(uint16_t maxValue);
  private:
    ILI9341_t3n        *_tft;
    uint16_t          *_buffer;
    //uint16_t          _buffer[sizeof(double_needle_image)*2];
    uint16_t          _min;
    uint16_t          _max;
    uint16_t          _value;
    uint16_t          _x;
    uint16_t          _y;
    uint16_t          _oldValue;
    _sprite_struct    sp;
};

Sprite::Sprite() {};

void Sprite::initSprite(ILI9341_t3n *tft, uint16_t x, uint16_t y, uint16_t minValue, uint16_t maxValue, bool _double)
{
  // Construction based on a 1 D pixel array
  _tft = tft;
  _x = x;
  _y = y;

  sp.x = _x;
  sp.y = _y;
  sp.width = 3;
  //sp.width = 4;
  if (_double)
  {
    //sp.height = 8;
    sp.height = 9;
    sp.SpriteImage = double_needle_image;
  } else {
    sp.height = 4;
    sp.SpriteImage = needle_image;
  }
  sp.visible = 0;

  _min = minValue;
  _max = maxValue;
  _value = _min;
  _oldValue = _min;

  _buffer = (uint16_t*)malloc(sp.width * sp.height * sizeof(uint16_t));
}

// Show/hide the sprite
void Sprite::drawSprite()
{
  if (sp.visible)
  {
    //debug.printf("  Ler buffer x=%d y=%d , W=%d, h=%d\n", (int)sp.x, (int)sp.y, (int)sp.width, (int)sp.height);
    _tft->readRect(sp.x, sp.y, sp.width, sp.height, _buffer);
    //debug.printf("  Mostra marcador\n");
    _tft->writeRect(sp.x, sp.y, sp.width, sp.height, sp.SpriteImage);
  } else {
    //debug.printf("  Apaga marcador x=%d y=%d , W=%d, h=%d\n", (int)sp.x, (int)sp.y, (int)sp.width, (int)sp.height);
    _tft->writeRect(sp.x, sp.y, sp.width, sp.height, _buffer);
  }
}

void Sprite::showSprite()
{
  sp.visible = 1;
  drawSprite();
}

void Sprite::hideSprite()
{
  sp.visible = 0;
  drawSprite();
}

void Sprite::moveSprite(uint16_t value)
{
  uint8_t WasVisible = sp.visible;

  if (value != _oldValue)
  {
    _oldValue = value;

    if (WasVisible) hideSprite();

    value = constrain(value, _min, _max);
    sp.x = map(value, _min, _max, _x, _x + GAUGE_LENGTH);

    if (WasVisible) showSprite();
  }
}

// Move to the specified position
void Sprite::moveSprite(uint16_t x, uint16_t y)
{
  uint8_t WasVisible = sp.visible;
  if (WasVisible) hideSprite();
  sp.x = x;
  sp.y = y;
  if (WasVisible) showSprite();
}

void Sprite::setMax(uint16_t maxValue)
{
  _max = maxValue;
}


// Does the sprite contain the given point : sprite must be visible
uint8_t Sprite::withinSprite(uint16_t x, uint16_t y)
{
  if (sp.visible == 0)
    return 0;
  if ( (x >= sp.x) && ( x < (sp.x + sp.width) ) )
  {
    if ( (y >= sp.y) && ( y < (sp.y + sp.height) ) )
      return 1;
    else
      return 0;
  }
  else
    return 0;
}

#endif
