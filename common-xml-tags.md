# Common XML Tags to Look For

Based on the research and quickstart guide, these are the common XML tags that need to be fixed in MDX files:

- `<inertial>`, `</inertial>`
- `<link>`, `</link>`
- `<joint>`, `</joint>`
- `<robot>`, `</robot>`
- `<visual>`, `</visual>`
- `<collision>`, `</collision>`
- `<geometry>`, `</geometry>`
- `<origin>`, `</origin>`
- `<mass>`, `</mass>`
- `<inertia>`, `</inertia>`
- `<material>`, `</material>`
- `<axis>`, `</axis>`

## Fixing Strategy

### For Inline XML Tags
Replace raw XML tags within text with inline code formatting:

**Before:**
```
What is the purpose of the <inertial> tag in URDF?
```

**After:**
```
What is the purpose of the `<inertial>` tag in URDF?
```

### For XML Code Blocks
Ensure XML snippets are properly fenced with the xml language identifier:

**Before:**
```
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
    </inertial>
  </link>
</robot>
```

**After:**
```
```xml
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
    </inertial>
  </link>
</robot>
```
```