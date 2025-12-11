"""
User and UserProfile Pydantic Models

Data models for user authentication and personalization features.
"""

from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4
from pydantic import BaseModel, EmailStr, Field, field_validator


# ==========================================
# User Models
# ==========================================

class UserBase(BaseModel):
    """Base user model with common fields"""
    email: EmailStr


class UserCreate(UserBase):
    """Model for user registration"""
    password: str = Field(..., min_length=8, max_length=100)

    @field_validator('password')
    @classmethod
    def password_strength(cls, v: str) -> str:
        """Validate password strength"""
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.islower() for c in v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one digit')
        return v


class UserLogin(UserBase):
    """Model for user login"""
    password: str


class User(UserBase):
    """Complete user model from database"""
    id: UUID = Field(default_factory=uuid4)
    is_active: bool = True
    is_verified: bool = False
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    last_login: Optional[datetime] = None

    model_config = {
        "from_attributes": True
    }


class UserPublic(UserBase):
    """Public user model (no sensitive data)"""
    id: UUID
    is_active: bool
    is_verified: bool
    created_at: datetime

    model_config = {
        "from_attributes": True
    }


# ==========================================
# User Profile Models
# ==========================================

class SkillLevel(str):
    """Enum-like class for skill levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"


class PreferredLanguage(str):
    """Enum-like class for preferred languages"""
    ENGLISH = "en"
    URDU = "ur"


class UserProfileBase(BaseModel):
    """Base user profile model"""
    full_name: Optional[str] = Field(None, max_length=255)
    skill_level: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced|expert)$")
    technical_background: Optional[str] = None
    learning_goals: Optional[str] = None
    preferred_language: str = Field(default="en", pattern="^(en|ur)$")
    personalization_enabled: bool = True


class UserProfileCreate(UserProfileBase):
    """Model for creating user profile"""
    pass


class UserProfileUpdate(BaseModel):
    """Model for updating user profile (all fields optional)"""
    full_name: Optional[str] = Field(None, max_length=255)
    skill_level: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced|expert)$")
    technical_background: Optional[str] = None
    learning_goals: Optional[str] = None
    preferred_language: Optional[str] = Field(None, pattern="^(en|ur)$")
    personalization_enabled: Optional[bool] = None


class UserProfile(UserProfileBase):
    """Complete user profile model from database"""
    id: UUID = Field(default_factory=uuid4)
    user_id: UUID
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Combined User + Profile Model
# ==========================================

class UserWithProfile(UserPublic):
    """User model with embedded profile"""
    profile: Optional[UserProfile] = None

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Authentication Response Models
# ==========================================

class Token(BaseModel):
    """JWT token response"""
    access_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds


class TokenPayload(BaseModel):
    """JWT token payload data"""
    sub: UUID  # user_id
    exp: int  # expiration timestamp
    iat: int  # issued at timestamp
    email: EmailStr
