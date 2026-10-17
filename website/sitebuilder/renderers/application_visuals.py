"""Small inline SVG scenes for the home page's application examples."""

from __future__ import annotations

_VISUALS = {
    "game-grid": """<svg class="application-visual application-visual--game-grid" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <path class="scene-grid" d="M24 38H336M24 68H336M24 98H336M24 128H336M54 28V136M84 28V136M114 28V136M144 28V136M174 28V136M204 28V136M234 28V136M264 28V136M294 28V136M324 28V136" />
  <text class="scene-label" x="20" y="20">GRID WORLD</text>
  <g fill="#31445d" opacity=".82">
    <rect x="58" y="42" width="24" height="20" rx="4" /><rect x="118" y="72" width="22" height="20" rx="4" />
    <rect x="178" y="102" width="22" height="20" rx="4" /><rect x="238" y="42" width="22" height="20" rx="4" />
    <rect x="298" y="72" width="22" height="20" rx="4" />
  </g>
  <path class="scene-route" data-motion-path="game" d="M30 126H96V98H155V126H218V70H278V38H330" />
  <path class="scene-route scene-route--active" d="M30 126H96V98H155V126H218V70H278V38H330" />
  <circle cx="30" cy="126" r="8" fill="#83f0cd" opacity=".16" /><circle cx="30" cy="126" r="3.5" fill="#a4f5df" />
  <circle cx="330" cy="38" r="10" fill="none" stroke="#ffb875" stroke-width="1.5" />
  <circle class="scene-pulse" cx="330" cy="38" r="7" fill="none" stroke="#ffb875" stroke-width="1.5" />
  <g class="scene-traveler scene-traveler--game-grid" data-motion-target="game" data-motion-duration="9"><circle class="scene-agent-dot" r="5" /></g>
</svg>""",
    "city-network": """<svg class="application-visual application-visual--city-network" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <text class="scene-label" x="20" y="20">CITY ROUTING</text>
  <g fill="none" stroke="rgba(139, 189, 255, .17)" stroke-width="9" stroke-linecap="round">
    <path d="M24 44H336M24 78H336M24 112H336M64 28V132M144 28V132M224 28V132M304 28V132" />
  </g>
  <g fill="none" stroke="rgba(14, 25, 41, .78)" stroke-width="4" stroke-linecap="round">
    <path d="M24 44H336M24 78H336M24 112H336M64 28V132M144 28V132M224 28V132M304 28V132" />
  </g>
  <path class="scene-route" data-motion-path="city" d="M25 112C80 112 94 78 143 78H220C252 78 253 40 335 40" />
  <path class="scene-route scene-route--active" d="M25 112C80 112 94 78 143 78H220C252 78 253 40 335 40" />
  <g class="scene-node"><circle cx="64" cy="112" r="3" /><circle cx="144" cy="78" r="3" /><circle cx="224" cy="78" r="3" /><circle cx="304" cy="40" r="3" /></g>
  <circle cx="25" cy="112" r="5" fill="#83f0cd" /><circle cx="335" cy="40" r="6" fill="#ffb875" />
  <g class="scene-traveler scene-traveler--city" data-motion-target="city" data-motion-duration="8" data-motion-rotate="true">
    <rect class="scene-car-body" x="-9" y="-5" width="18" height="10" rx="4" />
    <circle class="scene-wheel" cx="-5" cy="5" r="2" /><circle class="scene-wheel" cx="5" cy="5" r="2" />
  </g>
</svg>""",
    "shared-space": """<svg class="application-visual application-visual--shared-space" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <text class="scene-label" x="20" y="20">SHARED AISLE · SAFE ARRIVAL ORDER</text>
  <g fill="#192b41" stroke="#344a64" stroke-width="1">
    <rect x="28" y="34" width="66" height="27" rx="5" /><rect x="111" y="34" width="64" height="27" rx="5" />
    <rect x="205" y="34" width="64" height="27" rx="5" /><rect x="286" y="34" width="46" height="27" rx="5" />
    <rect x="28" y="99" width="66" height="27" rx="5" /><rect x="111" y="99" width="64" height="27" rx="5" />
    <rect x="205" y="99" width="64" height="27" rx="5" /><rect x="286" y="99" width="46" height="27" rx="5" />
  </g>
  <g stroke="rgba(156, 187, 222, .16)" stroke-width="2"><path d="M38 43H84M38 51H84M121 43H165M121 51H165M215 43H259M215 51H259M296 43H322M296 51H322M38 108H84M38 116H84M121 108H165M121 116H165M215 108H259M215 116H259M296 108H322M296 116H322" /></g>
  <path class="scene-route" data-motion-path="fleet-a" d="M26 80H130C169 80 169 72 195 72H334" />
  <path class="scene-route scene-route--active" d="M26 80H130C169 80 169 72 195 72H334" />
  <path class="scene-route scene-route--secondary" data-motion-path="fleet-b" d="M334 82H246C211 82 211 91 176 91H26" />
  <g class="scene-traveler scene-traveler--fleet" data-motion-target="fleet-a" data-motion-duration="8"><circle class="scene-agent-dot" r="6" /><text x="0" y="2.5" text-anchor="middle" fill="#10202a" font-size="7" font-weight="700">A</text></g>
  <g class="scene-traveler scene-traveler--fleet-b" data-motion-target="fleet-b" data-motion-duration="8" data-motion-wait="0.38,0.14"><circle class="scene-agent-dot" r="6" /><text x="0" y="2.5" text-anchor="middle" fill="#10202a" font-size="7" font-weight="700">B</text></g>
  <circle class="scene-pulse" cx="190" cy="81" r="7" fill="none" stroke="#ffb875" stroke-width="1.5" />
  <text class="scene-label" x="165" y="112">YIELD</text>
</svg>""",
    "car-maneuver": """<svg class="application-visual application-visual--car-maneuver" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <text class="scene-label" x="20" y="20">CURVATURE-CONSTRAINED MOTION</text>
  <path d="M24 120H336" stroke="rgba(139, 189, 255, .12)" stroke-width="1" stroke-dasharray="4 7" />
  <path class="scene-route" data-motion-path="car" d="M25 120C93 120 111 105 140 80C175 50 208 43 240 67C270 91 290 116 335 116" />
  <path class="scene-route scene-route--active" d="M25 120C93 120 111 105 140 80C175 50 208 43 240 67C270 91 290 116 335 116" />
  <rect class="scene-hazard" x="168" y="91" width="39" height="25" rx="6" />
  <path d="M177 104h21" stroke="#ffb875" stroke-width="2" stroke-linecap="round" />
  <circle cx="25" cy="120" r="4" fill="#83f0cd" /><circle cx="335" cy="116" r="5" fill="#ffb875" />
  <g class="scene-traveler scene-traveler--car" data-motion-target="car" data-motion-duration="6.5" data-motion-rotate="true">
    <rect class="scene-car-body" x="-12" y="-6" width="24" height="12" rx="5" />
    <path d="M-6 -5l3-4h8l4 4" fill="#c6e4ff" stroke="#effff9" stroke-width=".8" />
    <circle class="scene-wheel" cx="-7" cy="6" r="2.3" /><circle class="scene-wheel" cx="7" cy="6" r="2.3" />
  </g>
</svg>""",
    "robotic-arm": """<svg class="application-visual application-visual--robotic-arm" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <text class="scene-label" x="20" y="20">WORKCELL · CONFIGURATION SPACE</text>
  <g class="scene-grid"><path d="M24 42H336M24 72H336M24 102H336M64 30V132M104 30V132M144 30V132M184 30V132M224 30V132M264 30V132M304 30V132" /></g>
  <rect x="66" y="46" width="222" height="85" rx="9" fill="none" stroke="rgba(139, 189, 255, .23)" stroke-width="1.5" />
  <path d="M79 128H279" stroke="#60778f" stroke-width="4" stroke-linecap="round" />
  <path d="M117 128L145 86L191 67L216 86" class="scene-arm" />
  <circle cx="117" cy="128" r="8" class="scene-joint" /><circle cx="145" cy="86" r="6" class="scene-joint" />
  <circle cx="191" cy="67" r="5" class="scene-joint" />
  <path class="scene-route" data-motion-path="package" d="M218 118C191 118 178 106 164 90C150 74 135 70 119 70" />
  <path class="scene-route scene-route--active" d="M218 118C191 118 178 106 164 90C150 74 135 70 119 70" />
  <g class="scene-traveler scene-traveler--package" data-motion-target="package" data-motion-duration="6"><rect class="scene-package" x="-5" y="-5" width="10" height="10" rx="2" /></g>
  <path d="M210 82l8 4-8 4M210 88l8 4-8 4" fill="none" stroke="#c7dcf4" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" />
</svg>""",
    "service-network": """<svg class="application-visual application-visual--service-network" viewBox="0 0 360 156" aria-hidden="true" focusable="false">
  <rect class="scene-backdrop" width="360" height="156" />
  <text class="scene-label" x="20" y="20">WEIGHTED SERVICE GRAPH</text>
  <g class="scene-route">
    <path d="M29 112L83 51L144 112L177 78L235 38L333 42M83 51L177 78L270 112L333 42M144 112L270 112M83 51L235 38" />
  </g>
  <path class="scene-route" data-motion-path="packet" d="M29 112L83 51L144 112L177 78L235 38L333 42" />
  <path class="scene-route scene-route--active" d="M29 112L83 51L144 112L177 78L235 38L333 42" />
  <g class="scene-node">
    <circle cx="29" cy="112" r="6" /><circle cx="83" cy="51" r="6" /><circle cx="144" cy="112" r="6" />
    <circle cx="177" cy="78" r="6" /><circle cx="235" cy="38" r="6" /><circle cx="270" cy="112" r="6" /><circle cx="333" cy="42" r="6" />
  </g>
  <g fill="#142a42" stroke="#8bbdff" stroke-width="1.2"><rect x="70" y="38" width="26" height="25" rx="5" /><rect x="222" y="25" width="26" height="25" rx="5" /><rect x="320" y="29" width="26" height="25" rx="5" /></g>
  <path d="M76 46h14M76 51h14M228 33h14M228 38h14M326 37h14M326 42h14" stroke="#a4f5df" stroke-width="1.2" stroke-linecap="round" />
  <g class="scene-traveler scene-traveler--packet" data-motion-target="packet" data-motion-duration="5"><circle r="4.5" fill="#a4f5df" stroke="#effff9" stroke-width="1.5" filter="drop-shadow(0 0 5px rgba(131, 240, 205, .8))" /></g>
</svg>""",
}

APPLICATION_VISUAL_IDS = frozenset(_VISUALS)


def render_application_visual(visual: str) -> str:
    """Return a known, decorative SVG scene selected by application data."""
    try:
        return _VISUALS[visual]
    except KeyError as exc:
        raise ValueError(f"Unknown application visual: {visual}") from exc
