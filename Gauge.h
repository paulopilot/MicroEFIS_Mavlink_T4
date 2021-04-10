/***************************************************
  Name
    Gauge.h
    
  Description
    Create gauge indicators

  Functions
    init()
    update()
    value()
    setDangerInterval()
    setWarningInterval()
    setNormalInterval()
    getInterval()

  Author
    Paulo Almeida
    falmeida.paulo@gmail.com
    https://www.youtube.com/channel/UC-kUryZXKOTKsRH2tHRS1NA

  Updates
 ****************************************************/
#ifndef GAUGE_H__
#define GAUGE_H__

//#include <ILI9341_t3n.h>

#define DEFAULT_INTERVAL    500

class Gauge
{
  public:
    Gauge();
    void              init( ILI9341_t3n *tft,
                            uint16_t cx,
                            uint16_t cy,
                            uint16_t length,
                            int32_t minValue,
                            int32_t maxValue,
                            uint16_t offset,
                            uint16_t needleColor,
                            uint16_t backColor);
    void              update(void);
    //void              value(int32_t val, uint32_t interval = DEFAULT_INTERVAL);
    void              value(int32_t val);
    void              setDangerInterval(int32_t valMin, int32_t valMax);
    void              setWarningInterval(int32_t valMin, int32_t valMax);
    void              setNormalInterval(int32_t valMin, int32_t valMax);
    uint32_t          getInterval();

  private:
    ILI9341_t3n       *_tft;
    int32_t           _minValue;
    int32_t           _maxValue;
    int32_t           _value;

    int32_t           _dangerValueMin;
    int32_t           _dangerValueMax;

    int32_t           _warningValueMin;
    int32_t           _warningValueMax;

    int32_t           _normalValueMin;
    int32_t           _normalValueMax;

    uint16_t          _cx;
    uint16_t          _cy;
    uint16_t          _osx;
    uint16_t          _osy;
    uint16_t          _lenght;

    uint16_t          _offset;

    uint32_t          us_delay;

    uint16_t          _backColor;
    uint16_t          _needleColor;

    uint16_t          _textColor;

    uint16_t          _dangerColor;
    uint16_t          _warningColor;
    uint16_t          _normalColor;

    int32_t           _oldValue;

    int16_t           _minAngle;
    int16_t           _maxAngle;
    float             _ltx;

    void              setText(int32_t val);

    uint32_t          _step;
    elapsedMicros     elapsed_us;
    //elapsedMicros     elapsed_ms;
    //IntervalTimer     myTimer;

};

Gauge::Gauge() {

};

      
/***************************************************************************************
** Function name:           init
** Description:             Initizalize object
** Parameters:              tft - Display Reference
**                          cx  - Center position x
**                          cy  - Center position y
**                          lenght - 
**                          minValue - minimum value
**                          maxValue - maximum valie
**                          offset - 
**                          needleColor
**                          backColor 
***************************************************************************************/
void Gauge::init( ILI9341_t3n *tft, uint16_t cx, uint16_t cy, uint16_t length, int32_t minValue, int32_t maxValue, uint16_t offset, uint16_t needleColor, uint16_t backColor)
{
  _tft = tft;
  _cx = cx;
  _cy = cy;
  _osx = cx;
  _osy = cy;
  _minValue = minValue;
  _maxValue = maxValue;
  _oldValue = _minValue - 1;
  _step = (_maxValue - _minValue) / 20;
  _offset = offset;
  _needleColor = needleColor;
  _backColor = backColor;

  _dangerValueMax = _maxValue;
  _dangerValueMin = _maxValue - (int32_t)((float)_maxValue * 0.1); // default 10% to max value

  _warningValueMax = _dangerValueMin;
  _warningValueMin = _maxValue - (int32_t)((float)_maxValue * 0.1); // default 10% to max value

  _normalValueMax = _warningValueMin;
  _normalValueMin = _minValue;

  _dangerColor = ILI9341_RED;
  _warningColor = ILI9341_YELLOW;
  _normalColor = ILI9341_WHITE;

  _textColor = _normalColor;

  _minAngle = -210;
  _maxAngle = 15;
  _lenght = length;

  us_delay = 0;

  //myTimer.begin(update, DEFAULT_INTERVAL);
}

/***************************************************************************************
** Function name:           getInterval
** Description:             Get interval 
** Parameters:              tft - Display Reference
**                          cx  - Center position x
**                          cy  - Center position y
**                          lenght - 
**                          minValue - minimum value
**                          maxValue - maximum valie
**                          offset - 
**                          needleColor
**                          backColor 
***************************************************************************************/
uint32_t Gauge::getInterval() {
  return us_delay;
}

/***************************************************************************************
** Function name:           update()
** Description:             Update object
***************************************************************************************/
void Gauge::update() {

  if (elapsed_us <= us_delay) return;
  //debug.printf("Elapsed=%d  Delay=%d\n", (int)elapsed_us, (int)us_delay);
  elapsed_us = 0;

  // Move the needle util new value reached
  if (_oldValue != _value) {

    if (us_delay == 0) _oldValue = _value; // Update immediately id delay is 0

    if (_oldValue < _value) {
      _oldValue = _oldValue + _step;
      if (_oldValue > _value) _oldValue = _value;
    } else {
      _oldValue = _oldValue - _step;
      if (_oldValue < _value) _oldValue = _value;
    }

    // Map value to angle
    float sdeg = map(_oldValue, _minValue, _maxValue, _minAngle, _maxAngle);

    // Calcualte tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    _tft->drawLine(_cx + _offset * _ltx - 1, _cy - _offset, _osx - 1, _osy, _backColor);
    _tft->drawLine(_cx + _offset * _ltx    , _cy - _offset, _osx    , _osy, _backColor);
    _tft->drawLine(_cx + _offset * _ltx + 1, _cy - _offset, _osx + 1, _osy, _backColor);

    // Store new needle end coords for next erase
    _ltx = tx;
    _osx = sx * _lenght + _cx;
    _osy = sy * _lenght + _cy;

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    _tft->drawLine(_cx + _offset * _ltx - 1, _cy - _offset, _osx - 1, _osy, _needleColor);
    _tft->drawLine(_cx + _offset * _ltx    , _cy - _offset, _osx    , _osy, _needleColor);
    _tft->drawLine(_cx + _offset * _ltx + 1, _cy - _offset, _osx + 1, _osy, _needleColor);

    if ((_value >= _dangerValueMin) && (_value <= _dangerValueMax))
    {
      _textColor = _dangerColor;
    } else {
      if ((_value >= _warningValueMin) && (_value <= _warningValueMax))
      {
        _textColor = _warningColor;
      } else {
        if ((_value >= _normalValueMin) && (_value <= _normalValueMax))
        {
          _textColor = _normalColor;
        } else {
          _textColor = _normalColor;
        }
      }
    }

    setText(_oldValue);

    // Slow needle down slightly as it approaches new postion
    if (abs(_oldValue - _value) < (10 * _step)) us_delay += us_delay / 2;
  }
}

/***************************************************************************************
** Function name:           value()
** Description:             Set new value
** Parameters:              val - Value
**                          interval  - Interval beeween updates
***************************************************************************************/
//void Gauge::value(int32_t val, uint32_t interval)
void Gauge::value(int32_t val)
{
  //us_delay = interval;

  if (val < _minValue) val = _minValue; // Limit value to emulate needle end stops
  if (val > _maxValue) val = _maxValue;

  _value =  val;
  update();

  /*
    // Move the needle util new value reached
    while (!(val == _oldValue)) {

      //sprintf (buf, "%4d", _oldValue);
      //_tft->setTextColor(_backColor, _backColor);
      //_tft->drawStringCenter(_cx, _cy + 14, buf, 1);

      if (_oldValue < val) {
          _oldValue = _oldValue + _step;
          if (_oldValue > val) _oldValue = val;
      } else {
          _oldValue = _oldValue - _step;
          if (_oldValue < val) _oldValue = val;
      }

      //if (_oldValue < val) _oldValue = _oldValue + _step ; else _oldValue = _oldValue - _step;
      if (ms_delay == 0) _oldValue = val; // Update immediately id delay is 0
      //_oldValue = val; // Update immediately id delay is 0

      // Map value to angle
      float sdeg = map(_oldValue, _minValue, _maxValue, _minAngle, _maxAngle);

      // Calcualte tip of needle coords
      float sx = cos(sdeg * 0.0174532925);
      float sy = sin(sdeg * 0.0174532925);

      // Calculate x delta of needle start (does not start at pivot point)
      float tx = tan((sdeg+90) * 0.0174532925);

      // Erase old needle image
      _tft->drawLine(_cx + _offset * _ltx - 1, _cy - _offset, _osx - 1, _osy, _backColor);
      _tft->drawLine(_cx + _offset * _ltx    , _cy - _offset, _osx    , _osy, _backColor);
      _tft->drawLine(_cx + _offset * _ltx + 1, _cy - _offset, _osx + 1, _osy, _backColor);


      // Store new needle end coords for next erase
      _ltx = tx;
      _osx = sx * _lenght + _cx;
      _osy = sy * _lenght + _cy;

      // Draw the needle in the new postion, magenta makes needle a bit bolder
      // draws 3 lines to thicken needle
      _tft->drawLine(_cx + _offset * _ltx - 1, _cy - _offset, _osx - 1, _osy, _needleColor);
      _tft->drawLine(_cx + _offset * _ltx    , _cy - _offset, _osx    , _osy, _needleColor);
      _tft->drawLine(_cx + _offset * _ltx + 1, _cy - _offset, _osx + 1, _osy, _needleColor);


      if ((val >= _dangerValueMin) && (val <= _dangerValueMax))
      {
          _textColor = _dangerColor;
      } else {
          if ((val >= _warningValueMin) && (val <= _warningValueMax))
          {
              _textColor = _warningColor;
          } else {
              if ((val >= _normalValueMin) && (val <= _normalValueMax))
              {
                  _textColor = _normalColor;
              } else {
                  _textColor = _normalColor;
              }
          }
      }

      //sprintf (buf, "%4d", _oldValue);
      //_tft->setTextColor(_textColor, _backColor);
      //_tft->drawStringCenter(_cx, _cy + 14, buf, 1);

      setText(_oldValue);

      // Slow needle down slightly as it approaches new postion
      if (abs(_oldValue - val) < (10 * _step)) ms_delay += ms_delay / 2;

      // Wait before next update
      //delay(ms_delay);
      delayMicroseconds(ms_delay);
    }

    //debug.printf ("%d us\n", soma / count);
  */
}

/***************************************************************************************
** Function name:           setText()
** Description:             Set text value
** Parameters:              val - text value
***************************************************************************************/
void Gauge::setText(int32_t val) {
  char buf[8];
  static int32_t oldTextValue;

  if (oldTextValue != val) {
    //_tft->setFont(defalt_font);

    sprintf (buf, "%4d", (int)oldTextValue);
    //_tft->setTextColor(_backColor, _backColor);
    //_tft->drawStringCenter(_cx, _cy + 14, buf, 1);
    _tft->setTextColor(_backColor, _backColor);
    _tft->setCursor(_cx, _cy + 14, true);
    _tft->printf("%s", buf);

    oldTextValue = val;

    sprintf (buf, "%4d", (int)val);
    _tft->setTextColor(_needleColor, _backColor);
    //_tft->drawStringCenter(_cx, _cy + 14, buf, 1, _textColor, _backColor);
    _tft->setTextColor(_textColor, _backColor);
    _tft->setCursor(_cx, _cy + 14, true);
    _tft->printf("%s", buf);
  }
}

/***************************************************************************************
** Function name:           setDangerInterval()
** Description:             Set min and max danger zone
** Parameters:              valMin - minimum value
**                          valMax - maximum value
***************************************************************************************/
void Gauge::setDangerInterval(int32_t valMin, int32_t valMax) {
  _dangerValueMin = valMin;
  _dangerValueMax = valMax;
}

/***************************************************************************************
** Function name:           setWarningInterval()
** Description:             Set min and max Warning zone
** Parameters:              valMin - minimum value
**                          valMax - maximum value
***************************************************************************************/
void Gauge::setWarningInterval(int32_t valMin, int32_t valMax) {
  _warningValueMin = valMin;
  _warningValueMax = valMax;
}

/***************************************************************************************
** Function name:           setNormalInterval()
** Description:             Set min and max Normal zone
** Parameters:              valMin - minimum value
**                          valMax - maximum value
***************************************************************************************/
void Gauge::setNormalInterval(int32_t valMin, int32_t valMax) {
  _normalValueMin = valMin;
  _normalValueMax = valMax;
}
#endif
