
```json
{
  "sn": "A0000",
  "fu": "2506031500",
  "cc": 5,
  "n": 0,
  "u": [
    {
      "i": 1,
      "t": "2506010930",
      "d": 1,
      "e": 0,
      "v": 0
    },
    {
      "i": 2,
      "t": "2506011800",
      "d": 2,
      "e": 0,
      "v": 0
    },
    {
      "i": 3,
      "t": "2506020830",
      "d": 1,
      "e": 0,
      "v": 0
    }
  ]
}
```

### Field Mapping:

* `"sn"` → Serial Number
* `"fu"` → First Use (YYMMDDHHMM)
* `"cc"` → Cycle Count
* `"n"` → Note Type

  * 0 = normal
  * 1 = practice only
  * 2 = scrap
  * 3 = other
* `"u"` → Usage Log Array

  * `"i"` = ID
  * `"t"` = Power-on Time (YYMMDDHHMM)
  * `"d"` = Device Type (1 = robot, 2 = charger)
  * `"e"` = Energy in kJ (0 for now)
  * `"v"` = Minimum Voltage (0 for now)

