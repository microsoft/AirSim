from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any

class LayoutElement(BaseModel):
    """
    Represents a physical element in the document layout (BLOCO 369).
    """
    type: str = Field(..., description="Element type (e.g., table, paragraph, title)")
    bbox: List[float] = Field(..., description="Bounding box [x0, y0, x1, y1]")
    page: int = Field(..., description="Page number")
    content: Optional[str] = Field(None, description="Extracted text content")
    confidence: float = Field(0.0, description="Extraction confidence score")

class ExtractedEntity(BaseModel):
    """
    Represents a semantic entity extracted from the document (BLOCO 369).
    """
    name: str = Field(..., description="Entity name")
    value: Any = Field(..., description="Extracted value")
    unit: Optional[str] = Field(None, description="Measurement unit")
    page: int = Field(..., description="Page where it was found")
    bbox: List[float] = Field(..., description="Exact location in the document")
    snippet: str = Field(..., description="Context snippet")
    confidence: float = Field(0.0, description="Confidence score (C)")
    omega: float = Field(0.0, description="Semantic leaf (omega)")

class DocumentExtraction(BaseModel):
    """
    Complete extraction result for a document.
    """
    document_hash: str
    entities: List[ExtractedEntity]
    layout: List[LayoutElement]
    psi: float = 0.73
    satoshi_cost: float = 7.27
    status: str = "VALIDATED"
